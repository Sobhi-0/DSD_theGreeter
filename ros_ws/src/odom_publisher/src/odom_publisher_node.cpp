#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "greeter_msgs/msg/encoder_ticks.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <functional>
#include <iterator>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <rclcpp/duration.hpp>
#include <string>
#include <utility>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class OdomPublisher : public rclcpp::Node {

public:
  OdomPublisher() : Node("odom_publisher") {
    RCLCPP_INFO(get_logger(), "odom_publisher node has been created.");

    // Parameters
    this->declare_parameter("encoder_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("odom_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("wheel_base", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("wheel_radius", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("ticks_per_rev", rclcpp::PARAMETER_INTEGER);

    std::string encoder_topic =
        this->get_parameter("encoder_topic").as_string();
    std::string odom_topic = this->get_parameter("odom_topic").as_string();
    wheel_base = this->get_parameter("wheel_base").as_double();
    wheel_radius = this->get_parameter("wheel_radius").as_double();
    ticks_per_rev = this->get_parameter("ticks_per_rev").as_int();

    // subscriber
    encoder_subscriber_ =
        this->create_subscription<greeter_msgs::msg::EncoderTicks>(
            encoder_topic, 10,
            std::bind(&OdomPublisher::encoder_callback, this, _1));

    // publisher
    // odom_publisher_ =
    //     this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);

    pose_publisher = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("encoder_pose", 10);
    twist_publisher = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("encoder_twist", 10);

    // initialize covariance of pose
    {
      int filler_value = 1;
      for (int i = 0; i < 36; i += 6) {
        pose.covariance.at(i) = filler_value;
      }
    }
    // initialize covariance of twist
    {
      int filler_value = 1;
      for (int i = 0; i < 36; i += 6) {
        twist.covariance.at(i) = filler_value;
      }
    }
  }

private:
  void encoder_callback(const greeter_msgs::msg::EncoderTicks::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "wheel encoder: l: '%d' r: '%d'", msg->left,
                msg->right);

    rclcpp::Duration time_diff = (rclcpp::Time)msg->header.stamp -
                                 (rclcpp::Time)encoder_last.header.stamp;

    // calculate velocity
    {
      double ticks_per_second =
          (int)(msg->left - encoder_last.left) / time_diff.seconds();
      RCLCPP_WARN(get_logger(),"%f",ticks_per_second);

      lwheel_vel =
          2 * M_PI * wheel_radius * ticks_per_second / this->ticks_per_rev;
    }
    {
      double ticks_per_second =
          (int)(msg->right - encoder_last.right) / time_diff.seconds();

      rwheel_vel =
          2 * M_PI * wheel_radius * ticks_per_second / this->ticks_per_rev;
    }

    calculate_twist();
    calculate_pose(time_diff);

    // update buffer
    encoder_last = *msg;

    // nav_msgs::msg::Odometry odom;
    // odom.header.stamp = msg->header.stamp;
    // odom.header.frame_id = "odom";
    // odom.pose = this->pose;
    // odom_publisher_->publish(odom);
    geometry_msgs::msg::PoseWithCovarianceStamped pose_stamped;
    pose_stamped.pose = pose;
    pose_stamped.header.frame_id = "base_link";
    pose_stamped.header.stamp = msg->header.stamp;
    pose_publisher->publish(pose_stamped);

    geometry_msgs::msg::TwistWithCovarianceStamped twist_stamped;
    twist_stamped.twist = twist;
    twist_stamped.header.frame_id = "base_link";
    twist_stamped.header.stamp = msg->header.stamp;
    twist_publisher->publish(twist_stamped);
  }

  void calculate_twist() {
    twist.twist.linear.x = (lwheel_vel + rwheel_vel) / 2;
    twist.twist.angular.z = (rwheel_vel - lwheel_vel) / wheel_base;
  }

  void calculate_pose(rclcpp::Duration time_diff) {
    pose.pose.position.x += twist.twist.linear.x * time_diff.seconds();
    pose.pose.position.y += twist.twist.linear.x * time_diff.seconds();
    tf2::Quaternion quat;
    tf2::fromMsg(pose.pose.orientation, quat);
    quat.setRPY(0,0, quat.getAngle() + twist.twist.angular.z * time_diff.seconds());
    quat.normalize();
    pose.pose.orientation = tf2::toMsg(quat);
  }

  double lwheel_vel;
  double rwheel_vel;
  double wheel_base;
  double wheel_radius;
  int ticks_per_rev;
  greeter_msgs::msg::EncoderTicks encoder_last;
  geometry_msgs::msg::TwistWithCovariance twist;
  geometry_msgs::msg::PoseWithCovariance pose;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_publisher;
  rclcpp::Subscription<greeter_msgs::msg::EncoderTicks>::SharedPtr
      encoder_subscriber_;
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomPublisher>());
  rclcpp::shutdown();

  return 0;
}
