# ROS
from typing import Dict
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from apriltag_msgs.msg import AprilTagDetection


class ApriltagMergeNode(Node):
    def __init__(self):
        super().__init__("apriltag_merge_node")

        # parameters
        self.declare_parameter(
            "allowed_ids", rclpy.Parameter.Type.INTEGER_ARRAY
        )  # TODO: change the Type to list
        self.declare_parameter("camera_frame", rclpy.Parameter.Type.STRING)

        self.allowed_ids = self.get_parameter("allowed_ids")
        self.camera_frame = str(self.get_parameter("camera_frame"))

        # subscribes to the topics of the apriltag_node
        self.detection_sub = self.create_subscription(
            AprilTagDetection, "/apriltag_detections", self.listener_callback, 5
        )
        self.marker_detections = []  # Poses

        # tf listener
        # TODO: create tf listener

        # publishers
        self.position = self.create_publisher(
            PoseWithCovarianceStamped, "/apriltag_position", 5
        )

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def listener_callback(self, msg):
        if msg.id in self.allowed_ids:
            pose_with_covariance_stamped = PoseWithCovarianceStamped()
            pose_with_covariance_stamped.header.frame_id = self.camera_frame
            pose_with_covariance_stamped.header.stamp = self.get_clock().now().to_msg()

            # read position and invert it
            # TODO: read positions of apriltags from tf

            # calculate the 6x6 covariance matrix
            goodness = msg.goodness
            # TODO: calculate the covariance from the "goodness"
            # pose_with_covariance_stamped.pose.covariance = # list of 36 float values

            self.marker_detections.append(pose_with_covariance_stamped)

    def timer_callback(self):
        try:
            current_position = self.marker_detections.pop()
        except IndexError:
            current_position = PoseWithCovarianceStamped()
            current_position.header.frame_id = self.camera_frame
            current_position.header.stamp = self.get_clock().now().to_msg()
            # current_position.pose.covariance = [0.0 for _ in range(0, 36)]

        self.marker_detections.clear()
        self.position.publish(current_position)


def main(args=None):
    rclpy.init(args=args)
    apriltag_merge_node = ApriltagMergeNode()
    rclpy.spin(apriltag_merge_node)  # run node until shutdown
    apriltag_merge_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
