#include <cstdio>
#include <geometry_msgs/msg/detail/point32__struct.hpp>
#include <geometry_msgs/msg/detail/polygon_stamped__struct.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class PathRecorderNode : public rclcpp::Node {
public:
  PathRecorderNode() : Node("minimal_publisher") {
    publisher_ =
        this->create_publisher<geometry_msgs::msg::PolygonStamped>("topic", 10);
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        std::bind(&PathRecorderNode::topic_callback, this, _1));
    path.header.frame_id = "odom";
  }

private:
  void topic_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg) {

    geometry_msgs::msg::Point32 point;
    point.x = msg->pose.pose.position.x;
    point.y = msg->pose.pose.position.y;
    point.z = msg->pose.pose.position.z;
    path.polygon.points.push_back(point);
    publisher_->publish(path);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  geometry_msgs::msg::PolygonStamped path;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathRecorderNode>());
  rclcpp::shutdown();
  return 0;
}
