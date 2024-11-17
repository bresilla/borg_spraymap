#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <string>

class PointCloudSaver : public rclcpp::Node
{
public:
  PointCloudSaver(const std::string& format)
  : Node("pointcloud_saver"), format_(format)
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/added/cloud", 10,
      std::bind(&PointCloudSaver::topic_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "PointCloudSaver node started, waiting for point cloud...");
    RCLCPP_INFO(this->get_logger(), "Saving point cloud as %s file.", format_.c_str());
  }

private:
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Convert the ROS2 PointCloud2 message to PCL PointCloud
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);

    // Create a PCL PointCloud object
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromPCLPointCloud2(pcl_pc2, cloud);

    bool success = false;
    std::string filename;

    if (format_ == "pcd")
    {
      filename = "saved_cloud.pcd";
      if (pcl::io::savePCDFileASCII(filename, cloud) == 0)
      {
        success = true;
      }
    }
    else if (format_ == "ply")
    {
      filename = "saved_cloud.ply";
      if (pcl::io::savePLYFileASCII(filename, cloud) == 0)
      {
        success = true;
      }
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Unsupported file format: %s", format_.c_str());
    }

    if (success)
    {
      RCLCPP_INFO(this->get_logger(), "Saved point cloud to %s", filename.c_str());
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to save point cloud to %s", filename.c_str());
    }

    // Exit the node
    rclcpp::shutdown();
  }

  std::string format_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  // Initialize the ROS 2 system
  rclcpp::init(argc, argv);

  // Default format is 'ply'
  std::string format = "ply";

  // Parse command-line arguments
  if (argc > 1)
  {
    if (std::string(argv[1]) == "--format" && argc > 2)
    {
      format = argv[2];
      if (format != "pcd" && format != "ply")
      {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid format specified: %s", format.c_str());
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Supported formats are 'pcd' and 'ply'.");
        return 1;
      }
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: ros2 run pointcloud_saver pointcloud_saver_node [--format pcd|ply]");
      return 1;
    }
  }

  // Create the node with the specified format
  auto node = std::make_shared<PointCloudSaver>(format);

  // Spin the node
  rclcpp::spin(node);

  // Shutdown ROS 2
  rclcpp::shutdown();
  return 0;
}
