#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <vector>
#include <cmath>
#include <iostream>

#include <fmt/format.h>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <pcl_ros/transforms.hpp>

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

using namespace std::chrono_literals;

class CloudAdder: public rclcpp::Node{
    private:
        size_t count_;
        int step = 10;
        std::vector<std::vector<float>> added_array_;
        sensor_msgs::msg::PointCloud2 point_msg_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        message_filters::Subscriber<sensor_msgs::msg::PointCloud2> point_sub_;
        message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
        std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>> sync_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr corr_publisher_;

    public:
            CloudAdder(): Node("minimal_publisher"), count_(0) {
            publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
            point_sub_.subscribe(this, "/all/cloud");
            odom_sub_.subscribe(this, "/fbot/loc/odom");

            sync_ = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>>(10);
            sync_->connectInput(point_sub_, odom_sub_);
            sync_->registerCallback(std::bind(&CloudAdder::alt_callback, this, std::placeholders::_1, std::placeholders::_2));

            cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/added/cloud", 10);
            corr_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/corr/cloud", 10);
        }

    private:
        void alt_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& point_msg, const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg) {
            RCLCPP_INFO(this->get_logger(), "Received point cloud and odometry");
            geometry_msgs::msg::Point position = odom_msg->pose.pose.position;
            // std::array<float, 4> translation = {position.x, position.y, position.z, 1};
            Eigen::Vector4f translation_e = {position.x, position.y, position.z, 1};

            geometry_msgs::msg::Quaternion orientation = odom_msg->pose.pose.orientation;
            // std::array<float, 4> rotation = {orientation.x, orientation.y, orientation.z, orientation.w};
            Eigen::Vector4f rotation_e = {orientation.x, orientation.y, orientation.z, orientation.w};

            // auto transformation_matrix = compose_transformation_matrix(translation, rotation);
            auto transformation_matrix = compose_transformation_matrix(translation_e, rotation_e);
            sensor_msgs::PointCloud2ConstIterator<float> pt(*point_msg, "x");

            for (int i =0; pt != pt.end(); ++i, ++pt){
                if (i % step != 0) { continue; }
                Eigen::Vector4f point = {pt[0], pt[1], pt[2], 1};
                Eigen::Vector4f transformed_point = transformation_matrix * point;
                added_array_.push_back({transformed_point[0], transformed_point[1], transformed_point[2]});
            }
            sensor_msgs::msg::PointCloud2 added_cloud = cloud_maker(added_array_);
            cloud_publisher_->publish(added_cloud);
        }

        sensor_msgs::msg::PointCloud2 cloud_maker(const std::vector<std::vector<float>>& cloud_array) {
            sensor_msgs::msg::PointCloud2 added_cloud;
            added_cloud.header.frame_id = "fbot/map";
            added_cloud.header.stamp = this->now();
            added_cloud.height = 1;
            added_cloud.width = cloud_array.size();
            added_cloud.is_dense = false;
            added_cloud.is_bigendian = false;
            added_cloud.point_step = 12;
            added_cloud.row_step = added_cloud.point_step * added_cloud.width;

            // Define PointFields
            sensor_msgs::msg::PointField x_field, y_field, z_field;
            x_field.name = "x";
            x_field.offset = 0;
            x_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
            x_field.count = 1;
            y_field.name = "y";
            y_field.offset = 4;
            y_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
            y_field.count = 1;
            z_field.name = "z";
            z_field.offset = 8;
            z_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
            z_field.count = 1;
            added_cloud.fields = {x_field, y_field, z_field};

            // Resize data
            added_cloud.data.resize(added_cloud.row_step);

            // Iterate through cloud_array and populate PointCloud2
            sensor_msgs::PointCloud2Iterator<float> iter_x(added_cloud, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(added_cloud, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(added_cloud, "z");
            for (size_t i = 0; i < cloud_array.size(); ++i) {
                *iter_x = static_cast<float>(cloud_array[i][0]);
                *iter_y = static_cast<float>(cloud_array[i][1]);
                *iter_z = static_cast<float>(cloud_array[i][2]);
                ++iter_x; ++iter_y; ++iter_z;
            }

            return added_cloud;
        }

        Eigen::Matrix4f compose_transformation_matrix(const Eigen::Vector4f& translation, const Eigen::Vector4f& rotation) {
            Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
            // Set the translation components
            transformation_matrix.block<3, 1>(0, 3) = translation.head<3>();
            // Extract the rotation components
            float x = rotation[0], y = rotation[1], z = rotation[2], w = rotation[3];
            // Calculate the rotation matrix
            Eigen::Matrix3f rotation_matrix;
            rotation_matrix << 1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w,
                            2 * x * y + 2 * z * w, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w,
                            2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x * x - 2 * y * y;
            // Assign the rotation matrix to the appropriate block in the transformation matrix
            transformation_matrix.block<3, 3>(0, 0) = rotation_matrix;
            return transformation_matrix;
        }

        Eigen::Matrix4f compose_transformation_matrix(std::array<float, 4> translation, std::array<float, 4> rotation) {
            std::array<std::array<float, 4>, 4> transformation_matrix;
            // Initialize the transformation matrix as an identity matrix
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    transformation_matrix[i][j] = (i == j) ? 1.0 : 0.0;
                }
            }
            // Set the translation components
            for (int i = 0; i < 3; ++i) {
                transformation_matrix[i][3] = translation[i];
            }
            // Set the rotation components
            float x = rotation[0], y = rotation[1], z = rotation[2], w = rotation[3];
            std::array<std::array<float, 3>, 3> rotation_matrix = {{
                {1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w},
                {2 * x * y + 2 * z * w, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w},
                {2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x * x - 2 * y * y}
            }};
            // Copy the rotation matrix into the transformation matrix
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    transformation_matrix[i][j] = rotation_matrix[i][j];
                }
            }
            return Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::RowMajor>>(transformation_matrix.data()->data());
        }
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CloudAdder>());
    rclcpp::shutdown();
    return 0;
}
