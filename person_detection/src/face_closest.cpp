/*
    Jayden Grunde

    Node for rotating Quori's waist to face the closest object in a pointcloud.

*/ 

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <limits>
#include <cmath>
#include <algorithm>
#include <chrono>

class FaceClosest : public rclcpp::Node
{
public:
    FaceClosest()
        : Node("face_closest")
    {
        sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/detector", 10, std::bind(&FaceClosest::cloud_callback, this, std::placeholders::_1));
        sub_joint_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&FaceClosest::joint_callback, this, std::placeholders::_1));
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/quori/base_controller/cmd_vel", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&FaceClosest::publish_cmd, this));
        last_cloud_time_ = this->now();
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    float target_angle_ = 0.0;
    float turret_angle_ = 0.0;
    const float max_angular_speed_ = 0.5;
    bool got_cloud_ = false;
    bool got_joint_ = false;
    rclcpp::Time last_cloud_time_;
    const double cloud_timeout_sec_ = 1.0; // Stop if no cloud for 0.5s

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        float min_dist = std::numeric_limits<float>::max();
        float angle_to_closest = 0.0;

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
            float x = *iter_x;
            float y = *iter_y;
            float dist = std::sqrt(x * x + y * y);
            if (dist < min_dist)
            {
                min_dist = dist;
                angle_to_closest = std::atan2(y, x);
            }
        }

        if (min_dist < std::numeric_limits<float>::max())
        {
            target_angle_ = angle_to_closest;
            got_cloud_ = true;
            last_cloud_time_ = this->now();
        }
    }

    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.size() > 6) {
            turret_angle_ = msg->position[6];
            got_joint_ = true;
        }
    }

    void publish_cmd()
    {
        // Only publish if both callbacks have run at least once
        if (!got_cloud_ || !got_joint_) {
            return;
        }

        // If detector points have stopped, set velocity to 0
        if ((this->now() - last_cloud_time_).seconds() > cloud_timeout_sec_) {
            geometry_msgs::msg::Twist stop_cmd;
            pub_->publish(stop_cmd);
            return;
        }

        geometry_msgs::msg::Twist cmd;
        const float angle_tolerance = 0.1;

        // Error is the difference between target angle and turret angle
        float error = target_angle_ - turret_angle_ - M_PI_2;

        // Normalize error to [-pi, pi]
        while (error > M_PI) error -= 2 * M_PI;
        while (error < -M_PI) error += 2 * M_PI;

        RCLCPP_INFO(this->get_logger(), "Closest point angle: %.3f, Turret angle: %.3f", target_angle_, turret_angle_);

        if (std::abs(error) > angle_tolerance) {
            cmd.angular.z = error * 1.0;
            if (cmd.angular.z > max_angular_speed_) {
                cmd.angular.z = max_angular_speed_;
            } else if (cmd.angular.z < -max_angular_speed_) {
                cmd.angular.z = -max_angular_speed_;
            }
        } else {
            cmd.angular.z = 0.0;
        }

        pub_->publish(cmd);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FaceClosest>());
    rclcpp::shutdown();
    return 0;
}