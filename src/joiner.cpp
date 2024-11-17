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
const double pi = std::acos(-1);

class CloudJoiner: public rclcpp::Node{
    private:
        std::vector<std::vector<float>> added_array_;
        message_filters::Subscriber<sensor_msgs::msg::PointCloud2> point_sub_1;
        message_filters::Subscriber<sensor_msgs::msg::PointCloud2> point_sub_2;
        std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>>> sync_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
        int height_threshold = 0.15;
        int width_thresh_max = 3;
        int width_thresh_min = 0.8;

    public:
            CloudJoiner(): Node("pointcloud_joiner") {
                // timer_ = this->create_wall_timer(500ms, std::bind(&CloudAdder::timer_callback, this))
                point_sub_1.subscribe(this, "/two/cloud");
                point_sub_2.subscribe(this, "/three/cloud");

                sync_ = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>>>(10);
                sync_->connectInput(point_sub_1, point_sub_2);
                sync_->registerCallback(std::bind(&CloudJoiner::callback, this, std::placeholders::_1, std::placeholders::_2));

                cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/all/cloud", 10);
            }

    private:
        void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& point_msg_1, const sensor_msgs::msg::PointCloud2::ConstSharedPtr& point_msg_2) {
            RCLCPP_INFO(this->get_logger(), "Received two point clouds");
            std::array<float, 3> right_translation = {3.0, -0.2, 1.5};
            std::array<float, 3> right_rotation = {(pi/2), pi, (pi/2)};
            std::array<float, 3> left_translation = {3.0, 0.2, 1.5};
            std::array<float, 3> left_rotation = {-(pi/2), pi, (pi/2)};
            //swap left and right
            // sensor_msgs::msg::PointCloud2 transformed_cloud_1 = transform_cloud(point_msg_1, left_translation, left_rotation);
            std::vector<std::vector<float>> transformed_left_points = transformed_points(point_msg_1, left_translation, left_rotation);
            std::vector<std::vector<float>> transformed_right_points = transformed_points(point_msg_2, right_translation, right_rotation);

            std::vector<std::vector<float>> all_points;
            all_points.insert(all_points.end(), transformed_left_points.begin(), transformed_left_points.end());
            all_points.insert(all_points.end(), transformed_right_points.begin(), transformed_right_points.end());

            sensor_msgs::msg::PointCloud2 added_cloud = cloud_maker(all_points, std::string("fbot/base_link"));
            added_cloud.header.stamp = point_msg_1->header.stamp;
            cloud_publisher_->publish(added_cloud);
        }


        std::vector<std::vector<float>> transformed_points(
            const sensor_msgs::msg::PointCloud2::ConstSharedPtr& point_msg,
            std::array<float, 3> translation,
            std::array<float, 3> rotation) {

            float x = translation[0], y = translation[1], z = translation[2];
            float roll = rotation[0], pitch = rotation[1], yaw = rotation[2];

            Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
            transform(0, 3) = x;
            transform(1, 3) = y;
            transform(2, 3) = z;
            transform(0, 0) = std::cos(roll) * std::cos(pitch);
            transform(0, 1) = std::cos(roll) * std::sin(pitch) * std::sin(yaw) - std::sin(roll) * std::cos(yaw);
            transform(0, 2) = std::cos(roll) * std::sin(pitch) * std::cos(yaw) + std::sin(roll) * std::sin(yaw);
            transform(1, 0) = std::sin(roll) * std::cos(pitch);
            transform(1, 1) = std::sin(roll) * std::sin(pitch) * std::sin(yaw) + std::cos(roll) * std::cos(yaw);
            transform(1, 2) = std::sin(roll) * std::sin(pitch) * std::cos(yaw) - std::cos(roll) * std::sin(yaw);
            transform(2, 0) = -std::sin(pitch);
            transform(2, 1) = std::cos(pitch) * std::sin(yaw);
            transform(2, 2) = std::cos(pitch) * std::cos(yaw);

            std::vector<std::vector<float>> transformed_points;

            for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*point_msg, "x"), iter_y(*point_msg, "y"), iter_z(*point_msg, "z");
                 iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {

                float distance = std::sqrt(iter_x[0] * iter_x[0] + iter_y[0] * iter_y[0] + iter_z[0] * iter_z[0]);
                if (distance < 0.8) {
                    continue;
                }

                if (iter_x[0] > width_thresh_max || iter_x[0] < -width_thresh_max) {
                    continue;
                }

                Eigen::Vector4f point = {iter_x[0], iter_y[0], iter_z[0], 1};
                Eigen::Vector4f transformed_point = transform * point;

                if (transformed_point[2] < height_threshold) {
                    continue;
                }

                std::vector<float> transformed_point_array = {transformed_point[0], transformed_point[1], transformed_point[2]};
                transformed_points.push_back(transformed_point_array);
            }

            return transformed_points;
        }

        std::vector<std::vector<float>> transformed_points2(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& point_msg, std::array<float, 3> translation, std::array<float, 3> rotation){
            float x = translation[0], y = translation[1], z = translation[2];
            float roll = rotation[0], pitch = rotation[1], yaw = rotation[2];
            Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
            transform(0,3) = x;
            transform(1,3) = y;
            transform(2,3) = z;
            transform(0,0) = std::cos(roll)*std::cos(pitch);
            transform(0,1) = std::cos(roll)*std::sin(pitch)*std::sin(yaw) - std::sin(roll)*std::cos(yaw);
            transform(0,2) = std::cos(roll)*std::sin(pitch)*std::cos(yaw) + std::sin(roll)*std::sin(yaw);
            transform(1,0) = std::sin(roll)*std::cos(pitch);
            transform(1,1) = std::sin(roll)*std::sin(pitch)*std::sin(yaw) + std::cos(roll)*std::cos(yaw);
            transform(1,2) = std::sin(roll)*std::sin(pitch)*std::cos(yaw) - std::cos(roll)*std::sin(yaw);
            transform(2,0) = -std::sin(pitch);
            transform(2,1) = std::cos(pitch)*std::sin(yaw);
            transform(2,2) = std::cos(pitch)*std::cos(yaw);
            std::vector<std::vector<float>> transformed_points;
            for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*point_msg, "x"), iter_y(*point_msg, "y"), iter_z(*point_msg, "z"); iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z){
                if (iter_x[0] > width_thresh_max || iter_x[0] < -width_thresh_max) { continue; }
                Eigen::Vector4f point = {iter_x[0], iter_y[0], iter_z[0], 1};
                Eigen::Vector4f transformed_point = transform*point;
                if (transformed_point[2] < height_threshold) { continue; }
                std::vector<float> transformed_point_array = {transformed_point[0], transformed_point[1], transformed_point[2]};
                transformed_points.push_back(transformed_point_array);
            }
            return transformed_points;
        }

        sensor_msgs::msg::PointCloud2 cloud_maker(const std::vector<std::vector<float>>& cloud_array, std::string frame_id) {
            sensor_msgs::msg::PointCloud2 added_cloud;
            added_cloud.header.frame_id = frame_id;
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
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CloudJoiner>());
    rclcpp::shutdown();
    return 0;
}
