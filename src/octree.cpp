#include <chrono>
#include <ctime>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

using namespace std::chrono_literals;

class OctomapBuilder : public rclcpp::Node {
private:


    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_publisher_;

    std::shared_ptr<octomap::OcTree> octree_;
    //timer
    rclcpp::TimerBase::SharedPtr timer_;

    //point cloud message
    sensor_msgs::msg::PointCloud2::SharedPtr pcd_msg;

public:
    OctomapBuilder()
    : Node("octomap_builder")
    {
        // Subscribe to the point cloud topic
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/added/cloud", 10, std::bind(&OctomapBuilder::topic_callback, this, std::placeholders::_1));

        // Publisher for the octomap
        octomap_publisher_ = this->create_publisher<octomap_msgs::msg::Octomap>("octomap", 10);
        timer_ = this->create_wall_timer(60s, std::bind(&OctomapBuilder::timer_callback, this));

        // Initialize the octree with a specified resolution
        double resolution = 0.05; // 5 cm resolution
        octree_ = std::make_shared<octomap::OcTree>(resolution);
    }

private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
        pcd_msg = msg;
    }

    void timer_callback(){
        auto msg = pcd_msg;
        // Convert ROS PointCloud2 message to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *pcl_cloud);

        // Convert PCL PointCloud to octomap point cloud
        octomap::Pointcloud octo_cloud;
        for (const auto& point : pcl_cloud->points) {
            // Filter out NaN and infinite values
            if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
                octo_cloud.push_back(point.x, point.y, point.z);
            }
        }

        // Define the sensor origin (adjust if necessary)
        octomap::point3d sensor_origin(0.0f, 0.0f, 0.0f);

        // Insert the point cloud into the octree
        octree_->insertPointCloud(octo_cloud, sensor_origin);

        // Update inner occupancy values
        octree_->updateInnerOccupancy();

        // Convert the octree to a ROS Octomap message
        octomap_msgs::msg::Octomap octomap_msg;
        octomap_msg.header.frame_id = msg->header.frame_id;
        octomap_msg.header.stamp = msg->header.stamp;

        // Binary map (set to true for binary, false for full map)
        bool binary_map = true;

        if (binary_map) {
            // Convert to binary octomap message
            if (!octomap_msgs::binaryMapToMsg(*octree_, octomap_msg)) {
                RCLCPP_ERROR(this->get_logger(), "Error serializing OctoMap");
                return;
            }
        } else {
            // Convert to full octomap message
            if (!octomap_msgs::fullMapToMsg(*octree_, octomap_msg)) {
                RCLCPP_ERROR(this->get_logger(), "Error serializing OctoMap");
                return;
            }
        }

        // Publish the octomap message
        octomap_publisher_->publish(octomap_msg);

        // Log some information
        RCLCPP_INFO(this->get_logger(), "Octomap updated and published");
    }

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OctomapBuilder>());
    rclcpp::shutdown();
    return 0;
}
