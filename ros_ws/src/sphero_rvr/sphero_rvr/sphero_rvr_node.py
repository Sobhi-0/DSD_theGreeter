import os
import sys
import asyncio

# ROS
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist

# Sphero SDK
sys.path.append(
    os.path.abspath(
        os.path.join(
            os.path.dirname(__file__),
            "/ros_ws/src/sphero_rvr/sphero-sdk-raspberrypi-python",
        )
    )
)

from sphero_sdk import SpheroRvrAsync
from sphero_sdk import SerialAsyncDal
from sphero_sdk import SpheroRvrTargets
from sphero_sdk import DriveFlagsBitmask
from sphero_sdk import RawMotorModesEnum

# end of imports

loop = asyncio.get_event_loop()
rvr = SpheroRvrAsync(dal=SerialAsyncDal(loop))


class SpheroNode(Node):

    def __init__(self):
        super().__init__("sphero_rvr_node")

        # parameters
        self.declare_parameter("wheel_base_in_m", rclpy.Parameter.Type.DOUBLE)
        self.wheel_base_in_m = self.get_parameter("wheel_base_in_m")

        # subscriptions
        self.subscription = self.create_subscription(
            Twist, "cmd_vel", self.listener_callback, 5
        )

        # services
        self.wake_up_srv = self.create_service(
            Trigger, "sphero_wake_up", self.wake_up_service_callback
        )
        self.sleep_srv = self.create_service(
            Trigger, "sphero_sleep", self.sleep_service_callback
        )
        self.get_battery_srv = self.create_service(
            Trigger, "get_battery", self.get_battery_service_callback
        )

    async def listener_callback(self, msg):

        # calculate l and r speed from twist
        left_speed = msg.linear - (msg.angular * self.wheel_base_in_m / 2)
        right_speed = msg.linear + (msg.angular * self.wheel_base_in_m / 2)

        await rvr.raw_motors(
            left_mode=RawMotorModesEnum.reverse.value,
            left_speed=left_speed,  # Valid speed values are 0-255
            right_mode=RawMotorModesEnum.forward.value,
            right_speed=right_speed,  # Valid speed values are 0-255
        )

    async def wake_up_service_callback(self, _, response):
        # TODO: implement wake up service
        await rvr.wake()
        await asyncio.sleep(2)
        self.get_logger().info("woke up")
        response.success = True
        return response

    def sleep_service_callback(self, _, response):
        # TODO: implement sleep service
        self.get_logger().info("sleeping")
        response.success = True
        return response

    def get_battery_service_callback(self, _, response):
        # TODO: implement sleep service
        response.message = "100"
        response.success = True
        return response


async def main(args=None):
    rclpy.init(args=args)

    # sphero control
    sphero_rvr_node = SpheroNode()
    rclpy.spin(sphero_rvr_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sphero_rvr_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
