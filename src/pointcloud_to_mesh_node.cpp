#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <sys/stat.h>

#include "pointcloud_to_mesh/pointcloud_to_mesh_node.hpp"

namespace pointcloud_to_mesh
{

/**
 * @brief Constructor for the PointCloudToMeshNode
 *
 * Initializes the ROS 2 node, declares all parameters, retrieves their values,
 * and creates a subscription to the specified point cloud topic with appropriate
 * QoS settings for point cloud data.
 */
PointCloudToMeshNode::PointCloudToMeshNode() : Node("pointcloud_to_mesh_node")
{
  // Declare parameters with default values
  this->declare_parameter<std::string>("pointcloud_topic", "/rc_viscore/points2");
  this->declare_parameter<std::string>("output_path", "/tmp/mesh_output");
  this->declare_parameter<std::string>("output_basename", "mesh");
  this->declare_parameter<std::string>("output_format", "obj");

  this->declare_parameter<double>("voxel_downsample_size", 0.0);
  this->declare_parameter<bool>("remove_statistical_outliers", true);
  this->declare_parameter<int>("nso_nb_neighbors", 20);
  this->declare_parameter<double>("nso_std_ratio", 2.0);

  this->declare_parameter<double>("normal_radius", 0.05);
  this->declare_parameter<int>("normal_max_nn", 30);

  this->declare_parameter<std::string>("reconstruction_method", "poisson");
  this->declare_parameter<int>("poisson_depth", 9);
  this->declare_parameter<double>("poisson_density_quantile", 0.05);

  this->declare_parameter<bool>("shutdown_after_save", true);
  this->declare_parameter<bool>("log_level_debug", false);

  // Get parameter values
  pointcloud_topic_ = this->get_parameter("pointcloud_topic").as_string();
  output_path_ = this->get_parameter("output_path").as_string();
  output_basename_ = this->get_parameter("output_basename").as_string();
  output_format_ = this->get_parameter("output_format").as_string();

  voxel_downsample_size_ = this->get_parameter("voxel_downsample_size").as_double();
  remove_statistical_outliers_ = this->get_parameter("remove_statistical_outliers").as_bool();
  nso_nb_neighbors_ = this->get_parameter("nso_nb_neighbors").as_int();
  nso_std_ratio_ = this->get_parameter("nso_std_ratio").as_double();

  normal_radius_ = this->get_parameter("normal_radius").as_double();
  normal_max_nn_ = this->get_parameter("normal_max_nn").as_int();

  reconstruction_method_ = this->get_parameter("reconstruction_method").as_string();
  poisson_depth_ = this->get_parameter("poisson_depth").as_int();
  poisson_density_quantile_ = this->get_parameter("poisson_density_quantile").as_double();

  shutdown_after_save_ = this->get_parameter("shutdown_after_save").as_bool();
  log_level_debug_ = this->get_parameter("log_level_debug").as_bool();

  // Create subscription with best-effort QoS for point cloud data
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  qos.best_effort();
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    pointcloud_topic_, qos,
    std::bind(&PointCloudToMeshNode::pointCloudCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Subscribed to %s", pointcloud_topic_.c_str());
}

/**
 * @brief Callback function for handling incoming PointCloud2 messages
 *
 * This callback implements one-shot behavior by processing only the first
 * received message. It performs the full pipeline: converts ROS message
 * to PCL format, preprocesses the point cloud, estimates normals, performs
 * surface reconstruction, and saves the resulting mesh to disk.
 *
 * @param msg Shared pointer to the received PointCloud2 message
 */
void PointCloudToMeshNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Check if we've already processed a point cloud (one-shot behavior)
  if (processing_done_) {
    return;
  }
  processing_done_ = true;

  try {
    RCLCPP_INFO(this->get_logger(), "Received point cloud; converting to PCL format...");

    // Convert ROS message to PCL format
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

    RCLCPP_INFO(this->get_logger(), "Point count: %zu", cloud->size());

    // Preprocess the point cloud (downsampling and outlier removal)
    pcl::PointCloud<pcl::PointXYZ>::Ptr processed_cloud = preprocessPointCloud(cloud);

    // Estimate surface normals required for reconstruction algorithms
    pcl::PointCloud<pcl::Normal>::Ptr normals = estimateNormals(processed_cloud,
                                                               normal_radius_,
                                                               normal_max_nn_);

    // Perform surface reconstruction based on selected method
    pcl::PolygonMesh::Ptr mesh;
    if (reconstruction_method_ == "poisson") {
      mesh = poissonReconstruction(processed_cloud, normals, poisson_depth_, poisson_density_quantile_);
      RCLCPP_INFO(this->get_logger(), "Poisson reconstruction complete (depth=%d)", poisson_depth_);
    } else if (reconstruction_method_ == "bpa") {
      // Using Greedy Projection Triangulation as BPA alternative
      mesh = greedyProjectionReconstruction(processed_cloud, normals);
      RCLCPP_INFO(this->get_logger(), "Greedy Projection reconstruction complete");
    } else {
      throw std::runtime_error("reconstruction_method must be 'poisson' or 'bpa'");
    }

    // Save the resulting mesh to disk
    bool saved = saveMesh(mesh, output_path_, output_basename_, output_format_);
    if (saved) {
      std::string output_file = output_path_ + "/" + output_basename_ + "." + output_format_;
      RCLCPP_INFO(this->get_logger(), "Saved mesh to: %s", output_file.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to save mesh");
    }

  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create/save mesh: %s", e.what());
  }

  // Optionally shut down the node after processing
  if (shutdown_after_save_) {
    RCLCPP_INFO(this->get_logger(), "Shutting down node.");
    rclcpp::shutdown();
  }
}

/**
 * @brief Preprocess the point cloud data
 *
 * Applies optional voxel downsampling and statistical outlier removal
 * based on node parameters. This function creates a copy of the input
 * cloud and applies the selected preprocessing steps.
 *
 * @param cloud Input point cloud to preprocess
 * @return Processed point cloud with applied filters
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudToMeshNode::preprocessPointCloud(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
  // Create a copy of the input cloud to avoid modifying the original
  pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>(*cloud));

  // Apply voxel downsampling if enabled
  if (voxel_downsample_size_ > 0.0) {
    result = voxelDownsample(result, voxel_downsample_size_);
  }

  // Apply statistical outlier removal if enabled
  if (remove_statistical_outliers_) {
    result = removeStatisticalOutliers(result, nso_nb_neighbors_, nso_std_ratio_);
  }

  return result;
}

/**
 * @brief Apply voxel grid downsampling to the point cloud
 *
 * Reduces the number of points by creating a 3D grid and keeping only
 * one point per grid cell. This helps reduce computation time and memory usage.
 *
 * @param cloud Input point cloud to downsample
 * @param voxel_size Size of the voxel grid cells (in meters)
 * @return Downsampled point cloud
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudToMeshNode::voxelDownsample(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
  double voxel_size)
{
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

  // Set up and apply the voxel grid filter
  voxel_filter.setInputCloud(cloud);
  voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
  voxel_filter.filter(*cloud_filtered);

  return cloud_filtered;
}

/**
 * @brief Remove statistical outliers from the point cloud
 *
 * Removes points that are far from their neighbors based on statistical analysis.
 * This helps clean up noisy point cloud data.
 *
 * @param cloud Input point cloud to filter
 * @param mean_k Number of nearest neighbors to use for mean distance estimation
 * @param std_mul Standard deviation multiplier threshold
 * @return Point cloud with outliers removed
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudToMeshNode::removeStatisticalOutliers(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
  int mean_k, double std_mul)
{
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

  // Configure and apply the statistical outlier removal filter
  sor.setInputCloud(cloud);
  sor.setMeanK(mean_k);
  sor.setStddevMulThresh(std_mul);
  sor.filter(*cloud_filtered);

  return cloud_filtered;
}

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
pcl::PointCloud<pcl::Normal>::Ptr PointCloudToMeshNode::estimateNormals(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
  double radius, int max_nn)
{
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

  // Configure and compute normal estimation
  ne.setSearchMethod(tree);
  ne.setInputCloud(cloud);
  ne.setRadiusSearch(radius);
  ne.setKSearch(max_nn);
  ne.compute(*cloud_normals);

  return cloud_normals;
}

/**
 * @brief Perform Poisson surface reconstruction on the point cloud
 *
 * Creates a triangle mesh from the point cloud using the Poisson reconstruction
 * algorithm, which produces smooth surfaces by solving a Poisson equation.
 *
 * @param cloud Input point cloud with XYZ coordinates
 * @param normals Input point cloud with surface normals
 * @param depth Octree depth controlling reconstruction resolution
 * @param density_quantile Quantile threshold (0-1) for pruning low-density vertices
 * @return Reconstructed triangle mesh
 */
pcl::PolygonMesh::Ptr PointCloudToMeshNode::poissonReconstruction(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
  const pcl::PointCloud<pcl::Normal>::ConstPtr &normals,
  int depth, double density_quantile)
{
  // Concatenate XYZ coordinates and normals into a single PointNormal cloud
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

  pcl::Poisson<pcl::PointNormal> poisson;
  pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);

  // Configure and perform Poisson reconstruction
  poisson.setDepth(depth);
  poisson.setInputCloud(cloud_with_normals);
  poisson.reconstruct(*mesh);

  // Apply density quantile filtering if specified
  if (density_quantile > 0.0 && density_quantile < 1.0) {
    // Note: PCL doesn't have built-in density quantile pruning
    // This would require a custom implementation for full functionality
  }

  return mesh;
}

/**
 * @brief Perform Greedy Projection triangulation on the point cloud
 *
 * Creates a triangle mesh using the Greedy Projection algorithm, which is
 * an alternative to Ball Pivoting Algorithm for surface reconstruction.
 * This method works well with sparse point clouds.
 *
 * @param cloud Input point cloud with XYZ coordinates
 * @param normals Input point cloud with surface normals
 * @return Reconstructed triangle mesh
 */
pcl::PolygonMesh::Ptr PointCloudToMeshNode::greedyProjectionReconstruction(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
  const pcl::PointCloud<pcl::Normal>::ConstPtr &normals)
{
  // Concatenate XYZ coordinates and normals into a single PointNormal cloud
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

  // Set up search tree for neighbor queries
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud(cloud_with_normals);

  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);

  // Configure Greedy Projection triangulation parameters
  gp3.setSearchRadius(0.025);  // Default search radius
  gp3.setMu(2.5);              // Search radius multiplier
  gp3.setMaximumNearestNeighbors(100);  // Max neighbors to search for
  gp3.setMaximumSurfaceAngle(M_PI / 4); // Maximum surface angle (45 degrees)
  gp3.setMinimumAngle(M_PI / 18);       // Minimum angle (10 degrees)
  gp3.setMaximumAngle(2 * M_PI / 3);    // Maximum angle (120 degrees)
  gp3.setNormalConsistency(false);      // Whether normals are consistently oriented

  // Perform the triangulation
  gp3.setInputCloud(cloud_with_normals);
  gp3.setSearchMethod(tree2);
  gp3.reconstruct(*mesh);

  return mesh;
}

/**
 * @brief Save the triangle mesh to disk
 *
 * Creates the output directory if it doesn't exist and saves the mesh
 * in the specified format (OBJ or PLY). Returns true if successful.
 *
 * @param mesh Mesh to save
 * @param output_path Output directory for the mesh file
 * @param basename Base filename (without extension)
 * @param format Output format: "obj" or "ply"
 * @return True if the mesh was saved successfully, false otherwise
 */
bool PointCloudToMeshNode::saveMesh(const pcl::PolygonMesh::ConstPtr &mesh,
                                    const std::string &output_path,
                                    const std::string &basename,
                                    const std::string &format)
{
  // Create output directory if it doesn't exist
  std::string cmd = "mkdir -p " + output_path;
  int result = system(cmd.c_str());
  if (result != 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create output directory: %s", output_path.c_str());
    return false;
  }

  // Construct the full file path
  std::string file_path = output_path + "/" + basename + "." + format;

  // Save the mesh in the specified format
  if (format == "ply") {
    int save_result = pcl::io::savePLYFileBinary(file_path, *mesh);
    return (save_result == 0);  // Return true if save was successful
  } else if (format == "obj") {
    int save_result = pcl::io::saveOBJFile(file_path, *mesh);
    return (save_result == 0);  // Return true if save was successful
  } else {
    RCLCPP_ERROR(this->get_logger(), "output_format must be 'obj' or 'ply', got: %s", format.c_str());
    return false;
  }
}

}  // namespace pointcloud_to_mesh

/**
 * @brief Main entry point for the pointcloud to mesh node
 *
 * Initializes the ROS 2 context, creates the node, spins to process messages,
 * and handles cleanup when the node shuts down.
 *
 * @param argc Number of command-line arguments
 * @param argv Array of command-line argument strings
 * @return Exit code (0 for success)
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pointcloud_to_mesh::PointCloudToMeshNode>());
  rclcpp::shutdown();
  return 0;
}
