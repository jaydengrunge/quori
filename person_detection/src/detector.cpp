/*
    Jayden Grunde

    Node for clustering points from a LaserScan message using DBSCAN.

*/ 


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/header.hpp"
#include <vector>
#include <cmath>
#include <limits>
#include <Eigen/Dense>
#include "dbscan/dbscan.hpp"

class Detector : public rclcpp::Node
{
public:
    Detector()
        : Node("detector"), eps_(0.5), min_samples_(3)
    {
        // Subscription to the "filtered_scan" topic
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "filtered_scan", 10, std::bind(&Detector::callback, this, std::placeholders::_1));

        // Publisher to the "detector" topic
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("detector", 10);
    }

private:
    void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // // Log the frame_id for debugging
        // RCLCPP_INFO(this->get_logger(), "Received LaserScan message with frame_id: %s", msg->header.frame_id.c_str());

        std::vector<Eigen::Vector2f> points;

        // Convert LaserScan to Cartesian coordinates (x, y)
        float angle = msg->angle_min;
        for (const auto &range : msg->ranges)
        {
            if (std::isfinite(range))
            {
                float x = range * std::cos(angle);
                float y = range * std::sin(angle);
                points.emplace_back(x, y);
            }
            angle += msg->angle_increment;
        }

        if (!points.empty())
        {
            // Convert points to a format compatible with the DBSCAN API
            std::vector<point2> dbscan_points;
            for (const auto &point : points)
            {
                dbscan_points.push_back({point.x(), point.y()});
            }

            // Perform DBSCAN clustering
            auto clusters = dbscan(std::span<const point2>(dbscan_points), eps_, min_samples_);

            // Extract cluster centroids
            std::vector<std::array<float, 3>> cluster_centroids;
            for (const auto &cluster : clusters)
            {
                if (!cluster.empty())
                {
                    Eigen::Vector2f centroid = Eigen::Vector2f::Zero();
                    for (size_t index : cluster)
                    {
                        centroid += Eigen::Vector2f(dbscan_points[index].x, dbscan_points[index].y);
                    }
                    centroid /= cluster.size();
                    cluster_centroids.push_back({centroid.x(), centroid.y(), 0.0f});
                }
            }

            // Publish cluster centroids as PointCloud2
            if (!cluster_centroids.empty())
            {
                auto point_cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
                point_cloud_msg->header.stamp = msg->header.stamp; // Use LaserScan's timestamp
                point_cloud_msg->header.frame_id = msg->header.frame_id;

                sensor_msgs::PointCloud2Modifier modifier(*point_cloud_msg);
                modifier.setPointCloud2FieldsByString(1, "xyz");
                modifier.resize(cluster_centroids.size());

                sensor_msgs::PointCloud2Iterator<float> iter_x(*point_cloud_msg, "x");
                sensor_msgs::PointCloud2Iterator<float> iter_y(*point_cloud_msg, "y");
                sensor_msgs::PointCloud2Iterator<float> iter_z(*point_cloud_msg, "z");
                
                for (const auto &centroid : cluster_centroids)
                {
                    *iter_x = centroid[0]; ++iter_x;
                    *iter_y = centroid[1]; ++iter_y;
                    *iter_z = centroid[2]; ++iter_z;
                }
                
                publisher_->publish(*point_cloud_msg);
                // RCLCPP_INFO(this->get_logger(), "Published PointCloud2 with %zu clusters", cluster_centroids.size());
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "No valid points for clustering");
        }
    }

    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    const float eps_;
    const int min_samples_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Detector>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}