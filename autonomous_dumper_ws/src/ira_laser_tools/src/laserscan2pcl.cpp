#include <string.h>
#include <vector>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <laser_geometry/laser_geometry.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "pcl_ros/transforms.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std;
using namespace pcl;

using std::placeholders::_1;

class ScanToPCL : public rclcpp::Node
{
public:
    ScanToPCL();
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

private:
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
    laser_geometry::LaserProjection projector_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber;

    pcl::PCLPointCloud2 clouds;

    void laserscan_topic_parser();

    string destination_frame;
    string cloud_destination_topic;
    string laserscan_topic;
};

ScanToPCL::ScanToPCL() : Node("scan_to_pcl")
{
    this->declare_parameter<std::string>("destination_frame", "base_link");
    this->declare_parameter<std::string>("laserscan_topic", "/in_scan");
    this->declare_parameter<std::string>("cloud_destination_topic", "/out_cloud");

    this->get_parameter("destination_frame", destination_frame);
    this->get_parameter("cloud_destination_topic", cloud_destination_topic);
    this->get_parameter("laserscan_topic", laserscan_topic);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    this->laserscan_topic_parser();

    point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_destination_topic.c_str(), rclcpp::SensorDataQoS());
}

void ScanToPCL::laserscan_topic_parser()
{
    // workaround for std::bind https://github.com/ros2/rclcpp/issues/583
    std::function<void(const sensor_msgs::msg::LaserScan::SharedPtr)> callback =
        std::bind(
            &ScanToPCL::scanCallback,
            this, std::placeholders::_1);
    scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(laserscan_topic.c_str(), rclcpp::SensorDataQoS(), callback);
}

void ScanToPCL::scanCallback(sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    sensor_msgs::msg::PointCloud2 tmpCloud1, tmpCloud2;

    try
    {
        // Verify that TF knows how to transform from the received scan to the destination scan frame
        tf_buffer_->lookupTransform(scan->header.frame_id.c_str(), destination_frame.c_str(), scan->header.stamp, rclcpp::Duration(1, 0));
        projector_.transformLaserScanToPointCloud(scan->header.frame_id, *scan, tmpCloud1, *tf_buffer_, scan->range_max);
        pcl_ros::transformPointCloud(destination_frame.c_str(), tmpCloud1, tmpCloud2, *tf_buffer_);
    }
    catch (tf2::TransformException &ex)
    {
        return;
    }

    pcl_conversions::toPCL(tmpCloud2, clouds);

    pcl::PCLPointCloud2::Ptr input_cloud = boost::make_shared<pcl::PCLPointCloud2>(clouds);

    pcl::PCLPointCloud2::Ptr output_cloud(new pcl::PCLPointCloud2);

    // Copy the metadata
    output_cloud->header = input_cloud->header;
    output_cloud->height = input_cloud->height;
    output_cloud->width = input_cloud->width;
    output_cloud->is_bigendian = input_cloud->is_bigendian;
    // output_cloud->point_step = input_cloud->point_step - sizeof(uint32_t);                  // Adjust for the removed field
    // output_cloud->row_step = input_cloud->row_step - sizeof(uint32_t) * input_cloud->width; // Adjust row_step too
    output_cloud->point_step = input_cloud->point_step;                  // Adjust for the removed field
    output_cloud->row_step = input_cloud->row_step;                      // Adjust row_step too
    output_cloud->is_dense = input_cloud->is_dense;

    for (const auto &field : input_cloud->fields)
    {
        if (field.name != "index")
        {
            output_cloud->fields.push_back(field);
        }
    }

    // Allocate memory for the new data
    output_cloud->data.resize(output_cloud->row_step * output_cloud->height);

    // Copy over the data
    for (uint32_t i = 0; i < input_cloud->width * input_cloud->height; ++i)
    {
        memcpy(&output_cloud->data[i * output_cloud->point_step],
               &input_cloud->data[i * input_cloud->point_step],
               sizeof(float) * 3); // Copy x, y, z
        // Skip copying index
    }

    // Publish point cloud after publishing laser scan as for some reason moveFromPCL is causing getPointCloudAsEigen to
    // throw a segmentation fault crash
    std::shared_ptr<sensor_msgs::msg::PointCloud2> cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

    pcl_conversions::moveFromPCL(*output_cloud, *cloud_msg);

    point_cloud_publisher_->publish(*cloud_msg);

    // point_cloud_publisher_->publish(tmpCloud2);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<ScanToPCL>());

    rclcpp::shutdown();

    return 0;
}
