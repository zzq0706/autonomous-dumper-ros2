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

using namespace std;
using namespace pcl;

using std::placeholders::_1;

class PCLMerger : public rclcpp::Node
{
public:
    PCLMerger();
    void pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pl, std::string topic);
    rcl_interfaces::msg::SetParametersResult reconfigureCallback(const std::vector<rclcpp::Parameter> &parameters);

private:
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
    laser_geometry::LaserProjection projector_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> pcl_subscribers;
    std::vector<bool> clouds_modified;

    std::vector<pcl::PCLPointCloud2> clouds;
    std::vector<string> input_topics;

    void pcl_topic_parser();

    string destination_frame;
    string cloud_destination_topic;
    string cloud_topics;
};

PCLMerger::PCLMerger() : Node("pcl_multi_merger")
{
    this->declare_parameter<std::string>("destination_frame", "base_link");
    this->declare_parameter<std::string>("cloud_destination_topic", "/merged_cloud");
    this->declare_parameter<std::string>("cloud_topics", "");

    this->get_parameter("destination_frame", destination_frame);
    this->get_parameter("cloud_destination_topic", cloud_destination_topic);
    this->get_parameter("cloud_topics", cloud_topics);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    this->pcl_topic_parser();

    point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_destination_topic.c_str(), rclcpp::SensorDataQoS());

}


void PCLMerger::pcl_topic_parser()
{
    // cloud topics to subscribe
    std::map<std::string, std::vector<std::string>> topics;

    istringstream iss(cloud_topics);
    set<string> tokens;
    copy(istream_iterator<string>(iss), istream_iterator<string>(), inserter<set<string>>(tokens, tokens.begin()));
    std::vector<string> tmp_input_topics;

    while (!tokens.empty())
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for topics ...");
        sleep(1);

        topics = this->get_topic_names_and_types();

        for (const auto &topic_it : topics)
        {
            std::vector<std::string> topic_types = topic_it.second;

            if (std::find(topic_types.begin(), topic_types.end(), "sensor_msgs/msg/PointCloud2") != topic_types.end() && tokens.erase(topic_it.first) > 0)
            {
                tmp_input_topics.push_back(topic_it.first);
            }
        }
    }

    sort(tmp_input_topics.begin(), tmp_input_topics.end());
    std::vector<string>::iterator last = std::unique(tmp_input_topics.begin(), tmp_input_topics.end());
    tmp_input_topics.erase(last, tmp_input_topics.end());

    // Do not re-subscribe if the topics are the same
    if ((tmp_input_topics.size() != input_topics.size()) || !equal(tmp_input_topics.begin(), tmp_input_topics.end(), input_topics.begin()))
    {
        input_topics = tmp_input_topics;

        if (input_topics.size() > 0)
        {
            pcl_subscribers.resize(input_topics.size());
            clouds_modified.resize(input_topics.size());
            clouds.resize(input_topics.size());
            RCLCPP_INFO(this->get_logger(), "Subscribing to topics\t%ld", pcl_subscribers.size());
            for (std::vector<int>::size_type i = 0; i < input_topics.size(); ++i)
            {
                // workaround for std::bind https://github.com/ros2/rclcpp/issues/583
                std::function<void(const sensor_msgs::msg::PointCloud2::SharedPtr)> callback =
                    std::bind(
                        &PCLMerger::pclCallback,
                        this, std::placeholders::_1, input_topics[i]);
                pcl_subscribers[i] = this->create_subscription<sensor_msgs::msg::PointCloud2>(input_topics[i].c_str(), rclcpp::SensorDataQoS(), callback);
                clouds_modified[i] = false;
                cout << input_topics[i] << " ";
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Not subscribed to any topic.");
        }
    }
}

void PCLMerger::pclCallback(sensor_msgs::msg::PointCloud2::SharedPtr cloud, std::string topic)
{
    sensor_msgs::msg::PointCloud2 tmpCloud1, tmpCloud2;
    tmpCloud1 = *cloud;

    try
    {
        // Verify that TF knows how to transform from the received scan to the destination scan frame
        tf_buffer_->lookupTransform(cloud->header.frame_id.c_str(), destination_frame.c_str(), cloud->header.stamp, rclcpp::Duration(1, 0));
        pcl_ros::transformPointCloud(destination_frame.c_str(), tmpCloud1, tmpCloud2, *tf_buffer_);
    }
    catch (tf2::TransformException &ex)
    {
        return;
    }

    for (std::vector<int>::size_type i = 0; i < input_topics.size(); i++)
    {
        if (topic.compare(input_topics[i]) == 0)
        {
            pcl_conversions::toPCL(tmpCloud2, clouds[i]);

            // std::cout << "point_step: " << clouds[i].point_step;
            // for (const auto& field : clouds[i].fields)
            // {
            //     std::cout << "name: " << field.name
            //             << ", offset: " << field.offset
            //             << ", datatype: " << int(field.datatype)
            //             << ", count: " << field.count << std::endl;
            // }

            clouds_modified[i] = true;
        }
    }

    // Count how many clouds we have
    std::vector<int>::size_type totalClouds = 0;
    for (std::vector<int>::size_type i = 0; i < clouds_modified.size(); i++)
    {
        if (clouds_modified[i])
        {
            totalClouds++;
        }
    }

    // Go ahead only if all subscribed scans have arrived
    if (totalClouds == clouds_modified.size())
    {
        pcl::PCLPointCloud2 merged_cloud = clouds[0];
        clouds_modified[0] = false;

        for (std::vector<int>::size_type i = 1; i < clouds_modified.size(); i++)
        {
#if PCL_VERSION_COMPARE(>=, 1, 10, 0)
            merged_cloud += clouds[i];
#else
            pcl::concatenatePointCloud(merged_cloud, clouds[i], merged_cloud);
#endif

            clouds_modified[i] = false;
        }

        // for (const auto& field : merged_cloud.fields)
        // {
        //     std::cout << "name: " << field.name
        //             << ", offset: " << field.offset
        //             << ", datatype: " << int(field.datatype)
        //             << ", count: " << field.count << std::endl;
        // }

        std::shared_ptr<sensor_msgs::msg::PointCloud2> cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

        pcl_conversions::moveFromPCL(merged_cloud, *cloud_msg);

        point_cloud_publisher_->publish(*cloud_msg);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<PCLMerger>());

    rclcpp::shutdown();

    return 0;
}
