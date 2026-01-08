#ifndef POINTCLOUD_TO_MESH_NODE_HPP_
#define POINTCLOUD_TO_MESH_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/gp3.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include <pcl/PolygonMesh.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <string>
#include <vector>
#include <memory>

/**
 * @brief Namespace for the pointcloud to mesh conversion package
 */
namespace pointcloud_to_ply
{

/**
 * @brief ROS 2 node that converts a PointCloud2 stream into a triangle mesh.
 *
 * This node subscribes to a PointCloud2 topic, processes the point cloud data,
 * performs surface reconstruction using either Poisson or Greedy Projection
 * triangulation methods, and saves the resulting mesh to disk.
 *
 * The node is designed for one-shot mesh generation from static or semi-static
 * point clouds (e.g., for mapping, scanning, or reconstruction pipelines).
 *
 * The workflow includes:
 * 1. Subscribing to a PointCloud2 topic
 * 2. Converting to PCL format
 * 3. Preprocessing (downsampling, outlier removal)
 * 4. Estimating normals
 * 5. Reconstructing surface (Poisson or Greedy Projection)
 * 6. Saving mesh to disk and optionally shutting down
 */
class PointCloudToMeshNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for the PointCloudToMeshNode
   *
   * Initializes the ROS 2 node, declares parameters, sets up QoS profile,
   * and creates a subscription to the input point cloud topic.
   */
  PointCloudToMeshNode();

private:
  /**
   * @brief Callback function for handling incoming PointCloud2 messages
   *
   * This callback processes only the first received message (one-shot behavior),
   * converts the point cloud into a mesh, saves it to disk, and optionally
   * shuts down the node.
   *
   * @param msg Shared pointer to the received PointCloud2 message
   */
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  /**
   * @brief Preprocess the point cloud data
   *
   * Applies optional voxel downsampling and statistical outlier removal
   * based on node parameters.
   *
   * @param cloud Input point cloud to preprocess
   * @return Processed point cloud with applied filters
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr preprocessPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);

  /**
   * @brief Apply voxel grid downsampling to the point cloud
   *
   * Reduces the number of points by creating a 3D grid and keeping only
   * one point per grid cell.
   *
   * @param cloud Input point cloud to downsample
   * @param voxel_size Size of the voxel grid cells (in meters)
   * @return Downsampled point cloud
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxelDownsample(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
    double voxel_size);

  /**
   * @brief Remove statistical outliers from the point cloud
   *
   * Removes points that are far from their neighbors based on statistical analysis.
   *
   * @param cloud Input point cloud to filter
   * @param mean_k Number of nearest neighbors to use for mean distance estimation
   * @param std_mul Standard deviation multiplier threshold
   * @return Point cloud with outliers removed
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr removeStatisticalOutliers(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
    int mean_k, double std_mul);

  /**
   * @brief Estimate surface normals for the point cloud
   *
   * Computes normal vectors for each point in the cloud using K-nearest neighbors.
   * Normals are required for most surface reconstruction algorithms.
   *
   * @param cloud Input point cloud
   * @param radius Search radius for nearest neighbors
   * @param max_nn Maximum number of neighbors to use
   * @return Point cloud with estimated normals
   */
  pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
    double radius, int max_nn);

  /**
   * @brief Perform Poisson surface reconstruction on the point cloud
   *
   * Creates a triangle mesh from the point cloud using the Poisson reconstruction
   * algorithm, which produces smooth surfaces.
   *
   * @param cloud Input point cloud with XYZ coordinates
   * @param normals Input point cloud with surface normals
   * @param depth Octree depth controlling reconstruction resolution
   * @param density_quantile Quantile threshold (0-1) for pruning low-density vertices
   * @return Reconstructed triangle mesh
   */
  pcl::PolygonMesh::Ptr poissonReconstruction(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
    const pcl::PointCloud<pcl::Normal>::ConstPtr &normals,
    int depth, double density_quantile);

  /**
   * @brief Perform Greedy Projection triangulation on the point cloud
   *
   * Creates a triangle mesh using the Greedy Projection algorithm, which is
   * an alternative to Ball Pivoting Algorithm for surface reconstruction.
   *
   * @param cloud Input point cloud with XYZ coordinates
   * @param normals Input point cloud with surface normals
   * @return Reconstructed triangle mesh
   */
  pcl::PolygonMesh::Ptr greedyProjectionReconstruction(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
    const pcl::PointCloud<pcl::Normal>::ConstPtr &normals);

  /**
   * @brief Save the triangle mesh to disk
   *
   * Creates the output directory if it doesn't exist and saves the mesh
   * in the specified format (OBJ or PLY).
   *
   * @param mesh Mesh to save
   * @param output_path Output directory path
   * @param basename Base filename (without extension)
   * @param format Output format: "obj" or "ply"
   * @return True if the mesh was saved successfully, false otherwise
   */
  bool saveMesh(const pcl::PolygonMesh::ConstPtr &mesh,
                const std::string &output_path,
                const std::string &basename,
                const std::string &format);

  // ROS 2 subscription to PointCloud2 messages
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

  // Flag to ensure only one point cloud is processed (one-shot behavior)
  bool processing_done_{false};

  // Parameters
  std::string pointcloud_topic_;      // Topic to subscribe to for PointCloud2 messages
  std::string output_path_;           // Output directory for saved mesh files
  std::string output_basename_;       // Base filename for saved mesh files
  std::string output_format_;         // Output format: "obj" or "ply"
  double voxel_downsample_size_;      // Voxel size for downsampling (meters), disabled if <= 0
  bool remove_statistical_outliers_;  // Whether to remove statistical outliers
  int nso_nb_neighbors_;              // Number of neighbors for outlier detection
  double nso_std_ratio_;              // Standard deviation ratio threshold for outlier detection
  double normal_radius_;              // Search radius for normal estimation
  int normal_max_nn_;                 // Maximum number of neighbors for normal estimation
  std::string reconstruction_method_; // Reconstruction method: "poisson" or "bpa"
  int poisson_depth_;                 // Octree depth for Poisson reconstruction
  double poisson_density_quantile_;   // Density quantile for Poisson reconstruction
  bool shutdown_after_save_;          // Whether to shut down node after saving mesh
  bool log_level_debug_;              // Whether to enable debug logging
};

}  // namespace pointcloud_to_ply

#endif  // POINTCLOUD_TO_MESH_NODE_HPP_
