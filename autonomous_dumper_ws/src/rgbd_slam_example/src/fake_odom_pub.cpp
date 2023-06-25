#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>

class FakeOdomNode : public rclcpp::Node
{
public:
  FakeOdomNode() : Node("fake_odom_node")
  {
    // Create a publisher for nav_msgs/Odometry messages
    publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    // Create a tf2_ros::Buffer with a rclcpp::Clock
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

    // Create a tf2_ros::TransformListener
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


    // Create a timer to publish the transformed message periodically
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&FakeOdomNode::publishTransformedMessage, this));
  }

private:
  void publishTransformedMessage()
  {
    try
    {
      // Lookup the latest transform from '/base_link' to '/odom'
      geometry_msgs::msg::TransformStamped transform_stamped =
          tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);

      // Create an Odometry message
      nav_msgs::msg::Odometry odometry_msg;
      odometry_msg.header.frame_id = transform_stamped.header.frame_id;
      odometry_msg.header.stamp = transform_stamped.header.stamp;
      odometry_msg.child_frame_id = transform_stamped.child_frame_id;
      odometry_msg.pose.pose.position.x = transform_stamped.transform.translation.x;
      odometry_msg.pose.pose.position.y = transform_stamped.transform.translation.y;
      odometry_msg.pose.pose.position.z = transform_stamped.transform.translation.z;
      odometry_msg.pose.pose.orientation = transform_stamped.transform.rotation;

      // Publish the Odometry message
      publisher_->publish(odometry_msg);
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_WARN(this->get_logger(), "Failed to lookup transform: %s", ex.what());
    }
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeOdomNode>());
  rclcpp::shutdown();
  return 0;
}
