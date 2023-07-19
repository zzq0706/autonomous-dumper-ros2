#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <ncurses.h>

class KeyboardController : public rclcpp::Node
{
public:
  KeyboardController()
      : Node("keyboard_controller")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    initscr(); // Initialize ncurses window
    raw();
    keypad(stdscr, TRUE);
    noecho();     // Don't echo() while we getch
    timeout(100); // getch() non-blocking
    printw("Use 'w' to move forward\n");
    printw("Use 'a' to turn left\n");
    printw("Use 's' to move backward\n");
    printw("Use 'd' to turn right\n");
    printw("Use 'r' to reset velocity\n");
    printw("Press 'q' to quit\n");
    refresh();
  }

  void read_keyboard()
  {
    int ch = getch();
    auto msg = geometry_msgs::msg::Twist();
    if (ch != ERR)
    {
      switch (ch)
      {
      case 'w':
        vel += 0.2;
        msg.linear.x = vel;
        msg.angular.z = ang_vel;
        if (msg.linear.x > 1.0)
        {
          msg.linear.x = 1.0;
        }
        printw("current velocity is %f \n", vel);
        break;
      case 's':
        vel -= 0.2;
        msg.linear.x = vel;
        msg.angular.z = ang_vel;
        if (msg.linear.x < -1.0)
        {
          msg.linear.x = -1.0;
        }
        printw("current velocity is %f \n", vel);
        break;
      case 'a':
        ang_vel += 0.1;
        msg.linear.x = vel;
        msg.angular.z = ang_vel;
        if (msg.angular.z > 0.5)
        {
          msg.angular.z = 0.5;
        }
        printw("current angular velocity is %f \n", ang_vel);
        break;
      case 'd':
        ang_vel -= 0.1;
        msg.linear.x = vel;
        msg.angular.z = ang_vel;
        if (msg.angular.z < -0.5)
        {
          msg.angular.z = -0.5;
        }
        printw("current angular velocity is %f \n", ang_vel);
        break;
      case 'r':
        vel = 0.0;
        ang_vel = 0.0;
        msg.linear.x = vel;
        msg.angular.z = ang_vel;
        printw("current velocity is %f \n", vel);
        printw("current angular velocity is %f \n", ang_vel);
        break;
      case 'q':
        endwin(); // End ncurses window
        rclcpp::shutdown();
        exit(0);
      }
      publisher_->publish(msg);
    }
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  float vel = 0.0;
  float ang_vel = 0.0;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KeyboardController>();
  rclcpp::Rate loop_rate(10);
  while (rclcpp::ok())
  {
    node->read_keyboard();
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  endwin(); // End ncurses window
  rclcpp::shutdown();
  return 0;
}
