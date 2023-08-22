#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

class CameraInfoPublisher : public rclcpp::Node
{
public:
  CameraInfoPublisher()
    : Node("camera_info_publisher")
  {

    // Declare and set default values for parameters
    this->declare_parameter<std::string>("image_topic", "/depth_image");
	this->declare_parameter<std::string>("camera_info_topic", "/camera_info");
	this->get_parameter<std::string>("image_topic", image_topic_);
	this->get_parameter<std::string>("camera_info_topic", camera_info_topic_);

    // Set the field of view (fov_) parameter
    this->declare_parameter<double>("field_of_view", 70);
    this->get_parameter("field_of_view", fov_);

    publisher_ = create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic_, 10);

    // Subscribe to the image topic
    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, 10,
      [this](const sensor_msgs::msg::Image::SharedPtr img_msg) {
        // Create the CameraInfo message
        sensor_msgs::msg::CameraInfo info_msg;
        info_msg.width = img_msg->width;
        info_msg.height = img_msg->height;
        info_msg.distortion_model = "plumb_bob";
        info_msg.header.frame_id = img_msg->header.frame_id;
        info_msg.header.stamp = img_msg->header.stamp;

        double cx = img_msg->width * 0.5;
        double cy = img_msg->height * 0.5;
        double fx = 0.5 * img_msg->height / tan(fov_ * M_PI_2 / 180.0);
        double fy = fx;

        info_msg.k = {fx, 0, cx, 0, fy, cy, 0, 0, 1};
        info_msg.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        info_msg.d = {0, 0, 0, 0, 0};
        info_msg.p = {fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1};

        // Publish the CameraInfo message
        publisher_->publish(info_msg);
      });
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  
  std::string camera_info_topic_;
  std::string image_topic_;

  double fov_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraInfoPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

