import os
import sys
import asyncio

# ROS
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

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
from sphero_sdk import RvrStreamingServices

# end of imports

loop = asyncio.get_event_loop()
rvr = SpheroRvrAsync(dal=SerialAsyncDal(loop))


class SpheroNode(Node):

    async def __init__(self):
        super().__init__("sphero_rvr_node")

        # parameters
        self.declare_parameter("wheel_base_in_m", rclpy.Parameter.Type.DOUBLE)
        self.wheel_base_in_m = self.get_parameter("wheel_base_in_m")

        # subscriptions
        self.subscription = self.create_subscription(
            Twist, "cmd_vel", self.listener_callback, 5
        )

        # publishers
        self.imu_pub = self.create_publisher(Imu, "imu", 5)
        self.encoder_pub = self.create_publisher(String, "encoder", 5)

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

        await rvr.sensor_control.add_sensor_data_handler(
            service=RvrStreamingServices.imu, handler=self.imu_handler
        )

        await rvr.sensor_control.add_sensor_data_handler(
            service=RvrStreamingServices.encoders, handler=self.encoder_handler
        )

    async def imu_handler(self, imu_data):
        print("IMU data response: ", imu_data)
        ros_msg = Imu()
        ros_msg.header.stamp = self.get_clock().now().to_msg()
        ros_msg.header.frame_id = "imu_link"
        ros_msg.orientation.x = imu_data["QUATERNION"]["x"]
        ros_msg.orientation.y = imu_data["QUATERNION"]["y"]
        ros_msg.orientation.z = imu_data["QUATERNION"]["z"]
        ros_msg.orientation.w = imu_data["QUATERNION"]["w"]
        ros_msg.angular_velocity.x = imu_data["GYROSCOPE"]["x"]
        ros_msg.angular_velocity.y = imu_data["GYROSCOPE"]["y"]
        ros_msg.angular_velocity.z = imu_data["GYROSCOPE"]["z"]
        ros_msg.linear_acceleration.x = imu_data["ACCELEROMETER"]["x"]
        ros_msg.linear_acceleration.y = imu_data["ACCELEROMETER"]["y"]
        ros_msg.linear_acceleration.z = imu_data["ACCELEROMETER"]["z"]

        self.imu_pub.publish(str(imu_data))

    async def encode_handler(self, encoder_data):
        pass

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
