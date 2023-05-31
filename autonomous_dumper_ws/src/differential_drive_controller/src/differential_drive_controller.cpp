#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"

class DifferentialDriveController : public rclcpp::Node
{
public:
  DifferentialDriveController() : Node("differential_drive_controller")
  {
  	  
	// Declare and set default values for parameters
	this->declare_parameter<double>("wheel_distance", 1.865); // 0.9325 is the half of the wheel distance in unity
	this->declare_parameter<double>("track_radius", 0.3);

    this->get_parameter<double>("wheel_distance", wheel_distance);
    this->get_parameter<double>("track_radius", track_radius);
    
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&DifferentialDriveController::cmdVelCallback, this, std::placeholders::_1));

    left_wheel_speed_pub_ = this->create_publisher<std_msgs::msg::Float64>("/dumper/leftCrawler", 10);
    right_wheel_speed_pub_ = this->create_publisher<std_msgs::msg::Float64>("/dumper/rightCrawler", 10);
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // differential drive control logic
    double v = msg->linear.x;
    double w = msg->angular.z;
    
    // Limit maximum linear and angular speeds
    double linear_speed = std::min(std::max(v, -max_linear_speed_), max_linear_speed_);
    double angular_speed = std::min(std::max(w, -max_angular_speed_), max_angular_speed_);

    double left_crawler_speed_target = linear_speed - angular_speed * wheel_distance / 2.0; 
    double right_crawler_speed_target = linear_speed + angular_speed * wheel_distance / 2.0;

    // convert to track speed in unity
    double left_crawler_speed_unity = -left_crawler_speed_target / track_radius * 1.8 / 3.1415; // speed 1 m/s equals 100 degree/s of the track controller in unity
    double right_crawler_speed_unity = -right_crawler_speed_target / track_radius * 1.8 / 3.1415;

    // Publish left and right wheel speeds
    std_msgs::msg::Float64 left_wheel_speed_msg, right_wheel_speed_msg;
    left_wheel_speed_msg.data = left_crawler_speed_unity;  
    right_wheel_speed_msg.data = right_crawler_speed_unity; 
    left_wheel_speed_pub_->publish(left_wheel_speed_msg);
    right_wheel_speed_pub_->publish(right_wheel_speed_msg);
  }

  // Get parameter values
  double wheel_distance, track_radius;
  
  // Set speed limits
  const double max_linear_speed_ = 2.0;
  const double max_angular_speed_ = 1.0;
  
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
