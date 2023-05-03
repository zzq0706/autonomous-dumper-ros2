#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"

class DifferentialDriveController : public rclcpp::Node
{
public:
  DifferentialDriveController() : Node("differential_drive_controller")
  {
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&DifferentialDriveController::cmdVelCallback, this, std::placeholders::_1));

    left_wheel_speed_pub_ = this->create_publisher<std_msgs::msg::Float64>("/backHoe/leftCrawler", 10);
    right_wheel_speed_pub_ = this->create_publisher<std_msgs::msg::Float64>("/backHoe/rightCrawler", 10);
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // differential drive control logic
    double v = msg->linear.x;
    double w = msg->angular.z;

    double left_crawler_speed_target = v - w * 0.9325; // 0.9325 is the half of the wheel distance in unity
    double right_crawler_speed_target = v + w * 0.9325;

    // convert to track speed in unity
    double left_crawler_speed_unity = -left_crawler_speed_target * 6 / 3.1415;
    double right_crawler_speed_unity = -right_crawler_speed_target * 6 / 3.1415;

    // Publish left and right wheel speeds
    std_msgs::msg::Float64 left_wheel_speed_msg, right_wheel_speed_msg;
    left_wheel_speed_msg.data = left_crawler_speed_unity;  
    right_wheel_speed_msg.data = right_crawler_speed_unity; 
    left_wheel_speed_pub_->publish(left_wheel_speed_msg);
    right_wheel_speed_pub_->publish(right_wheel_speed_msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_wheel_speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_wheel_speed_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DifferentialDriveController>());
  rclcpp::shutdown();
  return 0;
}
