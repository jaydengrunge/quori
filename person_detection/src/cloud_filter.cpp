#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>

class PointCloud2Filter : public rclcpp::Node
{
public:
    PointCloud2Filter()
        : Node("pointcloud2_filter"), min_range_(0.1), max_range_(4.0)
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "astra_ros/devices/default/point_cloud", 10, std::bind(&PointCloud2Filter::callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_cloud", 10);
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert ROS PointCloud2 to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromROSMsg(*msg, *cloud);

        // Remove NaNs and points outside range
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
        for (const auto& pt : cloud->points) {
            float range = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
            if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z) &&
                range >= min_range_ && range <= max_range_) {
                cloud_filtered->points.push_back(pt);
            }
        }
        cloud_filtered->width = cloud_filtered->points.size();
        cloud_filtered->height = 1;
        cloud_filtered->is_dense = false;

        // Plane segmentation (RANSAC)
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.02); // Adjust as needed
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty()) {
            RCLCPP_WARN(this->get_logger(), "No plane found in point cloud.");
        } else {
            // Extract points not belonging to the plane (remove floor)
            pcl::ExtractIndices<pcl::PointXYZRGB> extract;
            extract.setInputCloud(cloud_filtered);
            extract.setIndices(inliers);
            extract.setNegative(true); // Remove the plane
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_no_floor(new pcl::PointCloud<pcl::PointXYZRGB>());
            extract.filter(*cloud_no_floor);

            // Convert back to ROS PointCloud2
            sensor_msgs::msg::PointCloud2 output;
            pcl::toROSMsg(*cloud_no_floor, output);
            output.header = msg->header;
            publisher_->publish(output);
            RCLCPP_INFO(this->get_logger(), "Published filtered cloud with %zu points (floor removed)", cloud_no_floor->points.size());
            return;
        }

        // If no plane found, publish the filtered cloud as is
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud_filtered, output);
        output.header = msg->header;
        publisher_->publish(output);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    const float min_range_;
    const float max_range_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloud2Filter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}