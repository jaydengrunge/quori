#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include "dbscan/dbscan.hpp"
#include <memory>
#include <mutex>

class PointCloudDetector : public rclcpp::Node
{
public:
    PointCloudDetector()
        : Node("pointcloud_detector"), eps_(0.1), min_samples_(40)
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "filtered_cloud", 10,
            std::bind(&PointCloudDetector::cloud_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_detector", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PointCloudDetector::timer_callback, this));
    }

private:
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_cloud_ = msg;
    }

    void timer_callback()
    {
        std::shared_ptr<sensor_msgs::msg::PointCloud2> msg;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!latest_cloud_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No point cloud received yet.");
                return;
            }
            msg = latest_cloud_;
        }

        std::vector<Eigen::Vector3f> points;
        sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
            float x = *iter_x;
            float y = *iter_y;
            float z = *iter_z;
            if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z))
            {
                points.emplace_back(x, y, z);
            }
        }

        if (!points.empty())
        {
            std::vector<point3> dbscan_points;
            for (const auto &point : points)
            {
                dbscan_points.push_back({point.x(), point.y(), point.z()});
            }

            auto clusters = dbscan(std::span<const point3>(dbscan_points), eps_, min_samples_);

            std::vector<std::array<float, 3>> cluster_centroids;
            for (const auto &cluster : clusters)
            {
                if (!cluster.empty())
                {
                    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
                    for (size_t index : cluster)
                    {
                        centroid += Eigen::Vector3f(dbscan_points[index].x, dbscan_points[index].y, dbscan_points[index].z);
                    }
                    centroid /= cluster.size();
                    cluster_centroids.push_back({centroid.x(), centroid.y(), centroid.z()});
                }
            }

            if (!cluster_centroids.empty())
            {
                auto point_cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
                point_cloud_msg->header.stamp = msg->header.stamp;
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
                RCLCPP_INFO(this->get_logger(), "Published PointCloud2 with %zu clusters", cluster_centroids.size());
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "No valid points for clustering");
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<sensor_msgs::msg::PointCloud2> latest_cloud_;
    std::mutex mutex_;

    const float eps_;
    const int min_samples_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}