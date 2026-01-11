/*
 * pointcloud_to_ply - Subscribe to a PointCloud2 topic, reconstruct a mesh, and save to .obj or .ply format
 *
 * Copyright (c) 2025 Alexandros PHILOTHEOU
 *
 * Licensed under the BSD License.
 * See LICENSE for details.
 */
#include <filesystem>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/poisson.h>

/**
 * @brief ROS 2 node that converts an incoming PointCloud2 message into a reconstructed mesh
 *        using Poisson surface reconstruction and saves it to disk in OBJ or PLY format.
 *
 * This node:
 * - Subscribes to a PointCloud2 topic.
 * - Optionally preprocesses the point cloud (voxel grid downsampling and statistical outlier removal).
 * - Estimates surface normals using local neighborhood search.
 * - Reconstructs a mesh via Poisson surface reconstruction.
 * - Saves the resulting mesh to a user-specified path and format.
 * - Optionally shuts down after processing the first point cloud.
 */
class PointCloudToPLYNode : public rclcpp::Node
{
  public:
    /**
     * @brief Constructor that initializes the node, declares ROS parameters, sets up QoS,
     *        and creates a subscription to the configured PointCloud2 topic.
     */
    PointCloudToPLYNode()
      : Node("pointcloud_to_ply_node"), done_(false)
    {
      // Declare parameters
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

      this->declare_parameter<int>("poisson_depth", 9);
      this->declare_parameter<double>("poisson_density_quantile", 0.05);

      this->declare_parameter<bool>("shutdown_after_save", true);

      // Get topic name
      std::string topic = this->get_parameter("pointcloud_topic").as_string();

      // Create QoS profile
      auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

      // Create subscription
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic, qos,
        std::bind(&PointCloudToPLYNode::pointcloud_callback, this, std::placeholders::_1));

      RCLCPP_INFO(this->get_logger(), "Subscribed to %s", topic.c_str());
    }

  private:
    /**
     * @brief Callback triggered when a PointCloud2 message is received.
     *        Performs mesh reconstruction and saves the result.
     *        Only processes the first message unless `shutdown_after_save` is false.
     *
     * @param msg Shared pointer to the incoming PointCloud2 message.
     */
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
      if (done_) {
        return;
      }

      done_ = true;

      try {
        RCLCPP_INFO(this->get_logger(), "Received point cloud; converting to PCL...");

        // Convert ROS PointCloud2 to PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        RCLCPP_INFO(this->get_logger(), "Point count: %zu", cloud->size());

        // Preprocess
        cloud = preprocess_pointcloud(cloud);

        // Estimate normals
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals = estimate_normals(cloud);

        // Reconstruct mesh using Poisson
        pcl::PolygonMesh mesh = poisson_reconstruction(cloud_with_normals);

        // Save mesh
        save_mesh(mesh);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create/save mesh: %s", e.what());
      }

      if (this->get_parameter("shutdown_after_save").as_bool()) {
        RCLCPP_INFO(this->get_logger(), "Shutting down node.");
        rclcpp::shutdown();
      }
    }

    /**
     * @brief Applies optional preprocessing steps to the input point cloud:
     *        - Voxel grid downsampling (if voxel_downsample_size > 0)
     *        - Statistical outlier removal (if enabled)
     *
     * @param cloud Input point cloud (XYZ only).
     * @return Processed point cloud.
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr preprocess_pointcloud(
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
      // Voxel downsampling
      double voxel_size = this->get_parameter("voxel_downsample_size").as_double();
      if (voxel_size > 0.0) {
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);
        voxel_filter.filter(*downsampled);
        cloud = downsampled;
        RCLCPP_INFO(this->get_logger(), "After downsampling: %zu points", cloud->size());
      }

      // Statistical outlier removal
      if (this->get_parameter("remove_statistical_outliers").as_bool()) {
        int nb_neighbors = this->get_parameter("nso_nb_neighbors").as_int();
        double std_ratio = this->get_parameter("nso_std_ratio").as_double();

        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(nb_neighbors);
        sor.setStddevMulThresh(std_ratio);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        sor.filter(*filtered);
        cloud = filtered;
        RCLCPP_INFO(this->get_logger(), "After outlier removal: %zu points", cloud->size());
      }

      return cloud;
    }

    /**
     * @brief Estimates surface normals for the input point cloud using local neighborhood search.
     *
     * @param cloud Input point cloud (XYZ only).
     * @return Point cloud with XYZ coordinates and associated normals (PointNormal type).
     */
    pcl::PointCloud<pcl::PointNormal>::Ptr estimate_normals(
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
      double normal_radius = this->get_parameter("normal_radius").as_double();
      int normal_max_nn = this->get_parameter("normal_max_nn").as_int();

      RCLCPP_INFO(this->get_logger(), "Estimating normals...");

      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
      ne.setInputCloud(cloud);
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
      ne.setSearchMethod(tree);
      ne.setRadiusSearch(normal_radius);
      ne.setKSearch(normal_max_nn);

      pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
      ne.compute(*normals);

      // Combine XYZ and normals
      pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(
        new pcl::PointCloud<pcl::PointNormal>);
      pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

      RCLCPP_INFO(this->get_logger(), "Normals estimated.");

      return cloud_with_normals;
    }

    /**
     * @brief Performs Poisson surface reconstruction on a point cloud with normals.
     *
     * Note: PCL's Poisson implementation does not support density-based vertex filtering,
     *       so the `poisson_density_quantile` parameter is only used for logging a warning.
     *
     * @param cloud_with_normals Input point cloud with normals.
     * @return Reconstructed polygon mesh.
     */
    pcl::PolygonMesh poisson_reconstruction(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals)
    {
      int depth = this->get_parameter("poisson_depth").as_int();
      double density_quantile = this->get_parameter("poisson_density_quantile").as_double();

      RCLCPP_INFO(this->get_logger(), "Performing Poisson reconstruction (depth=%d)...", depth);

      pcl::Poisson<pcl::PointNormal> poisson;
      poisson.setDepth(depth);
      poisson.setInputCloud(cloud_with_normals);

      pcl::PolygonMesh mesh;
      poisson.reconstruct(mesh);

      // Note: PCL's Poisson reconstruction doesn't directly expose density values
      // like Open3D does, so density-based filtering is not straightforward.
      // The mesh may contain some low-density artifacts at boundaries.
      if (density_quantile > 0.0 && density_quantile < 1.0) {
        RCLCPP_WARN(this->get_logger(),
          "Density-based vertex filtering is not available in PCL. "
          "Consider post-processing the mesh to remove boundary artifacts.");
      }

      RCLCPP_INFO(this->get_logger(), "Poisson reconstruction complete. Vertices: %zu, Polygons: %zu",
        mesh.cloud.width * mesh.cloud.height, mesh.polygons.size());

      return mesh;
    }

    /**
     * @brief Saves the reconstructed mesh to disk in the specified format (OBJ or PLY).
     *        Creates the output directory if it doesn't exist.
     *
     * @param mesh The mesh to save.
     */
    void save_mesh(const pcl::PolygonMesh& mesh)
    {
      std::string output_path = this->get_parameter("output_path").as_string();
      std::string basename = this->get_parameter("output_basename").as_string();
      std::string format = this->get_parameter("output_format").as_string();
      std::transform(format.begin(), format.end(), format.begin(), ::tolower);

      // Create output directory
      std::filesystem::create_directories(output_path);

      std::string filepath = output_path + "/" + basename + "." + format;

      if (format == "ply") {
        pcl::io::savePLYFile(filepath, mesh);
      }
      else if (format == "obj") {
        pcl::io::saveOBJFile(filepath, mesh);
      }
      else {
        throw std::runtime_error("output_format must be 'obj' or 'ply'");
      }

      RCLCPP_INFO(this->get_logger(), "Saved mesh to: %s", filepath.c_str());
    }

    // ROS 2 subscription to PointCloud2 topic
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

    // Flag to ensure only the first point cloud is processed
    bool done_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudToPLYNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
