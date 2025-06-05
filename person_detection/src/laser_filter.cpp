/*
    Jayden Grunde

    Node for filtering LaserScan messages based on range and angle limits.

*/ 

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>

class LaserFilter : public rclcpp::Node
{
public:
    LaserFilter()
        : Node("laser_filter"), min_range_(.5), max_range_(2.8),
          angle_min_(0), angle_max_(6.28) 
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&LaserFilter::callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("filtered_scan", 10);
    }

private:
    void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto filtered_msg = std::make_shared<sensor_msgs::msg::LaserScan>();

        // Copy metadata from the original message
        filtered_msg->header = msg->header;
        filtered_msg->angle_min = msg->angle_min;
        filtered_msg->angle_max = msg->angle_max;
        filtered_msg->angle_increment = msg->angle_increment;
        filtered_msg->time_increment = msg->time_increment;
        filtered_msg->scan_time = msg->scan_time;
        filtered_msg->range_min = msg->range_min;
        filtered_msg->range_max = msg->range_max;

        // Filter ranges based on min_range_, max_range_, and angle limits
        filtered_msg->ranges.reserve(msg->ranges.size());
        float angle = msg->angle_min;
        for (size_t i = 0; i < msg->ranges.size(); ++i, angle += msg->angle_increment)
        {
            if (angle >= angle_min_ && angle <= angle_max_)
            {
                float range = msg->ranges[i];
                if (range >= min_range_ && range <= max_range_)
                {
                    filtered_msg->ranges.push_back(range);
                }
                else
                {
                    filtered_msg->ranges.push_back(std::numeric_limits<float>::infinity());
                }
            }
            else
            {
                filtered_msg->ranges.push_back(std::numeric_limits<float>::infinity());
            }
        }

        // Copy intensities from the original message
        filtered_msg->intensities = msg->intensities;

        // Publish the filtered message
        publisher_->publish(*filtered_msg);

        // RCLCPP_INFO(this->get_logger(), "Published %zu range-filtered values (angle_min=%.2f, angle_max=%.2f)", 
        //     filtered_msg->ranges.size(), angle_min_, angle_max_);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;

    const float min_range_;
    const float max_range_;
    const float angle_min_;
    const float angle_max_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserFilter>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}