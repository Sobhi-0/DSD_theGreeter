import os
import sys
import time

# ROS
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
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

from sphero_sdk import SpheroRvrTargets
from sphero_sdk import DriveFlagsBitmask
from sphero_sdk import RawMotorModesEnum
from sphero_sdk import RvrStreamingServices

from sphero_sdk import SpheroRvrObserver
from sphero_sdk import RawMotorModesEnum

# end of imports

vel_lut = {
      0: 0,
     31: 0.2,
     63: 0.2,
     95: 0.2,
    127: 0.2,
    159: 0.2,
    191: 0.2,
    223: 0.2,
    255: 0.2
}

def lut_lookup(vel_m_per_s, intervall=32):
    lower_bound = (0, 0)    
    upper_bound = (0, 0)

    for key, value in vel_lut.items():
        if vel_m_per_s < value:
            upper_bound = (key, value)
            break
        lower_bound = (key, value)

    # interpolate
    if lower_bound[0] == upper_bound[0]:
        return lower_bound[0]
    else:
        vel_per_pwm = (upper_bound[0] - lower_bound[0]) / (upper_bound[1] - lower_bound[1])
        return lower_bound[0] + (vel_m_per_s - lower_bound[1]) / vel_per_pwm

rvr = SpheroRvrObserver()


class SpheroNode(Node):

    def __init__(self):
        super().__init__("sphero_rvr_node")

        # parameters
        self.declare_parameter("wheel_base_in_m", rclpy.Parameter.Type.DOUBLE)
        self.wheel_base_in_m = self.get_parameter("wheel_base_in_m").value

        # subscriptions
        self.subscription = self.create_subscription(
            Twist, "cmd_vel", self.listener_callback, 5
        )

        # publishers
        self.imu_pub = self.create_publisher(Imu, "imu", 5)
        self.lwheel_pub = self.create_publisher(Int32, "lwheel", 5)
        self.rwheel_pub = self.create_publisher(Int32, "rwheel", 5)

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

        rvr.wake()

        time.sleep(2)
        
        rvr.sensor_control.add_sensor_data_handler(
            service=RvrStreamingServices.imu,
            handler=self.imu_handler
        )
        rvr.sensor_control.add_sensor_data_handler(
            service=RvrStreamingServices.encoders,
            handler=self.encoder_handler
        )


        for k in rvr.sensor_control.supported_sensors:
            self.get_logger().info(k)

        rvr.sensor_control.start(interval=250)

        # await rvr.sensor_control.add_sensor_data_handler(
        #     service=RvrStreamingServices.imu, handler=self.imu_handler
        # )

        # await rvr.sensor_control.add_sensor_data_handler(
        #     service=RvrStreamingServices.encoders, handler=self.encoder_handler
        # )

    def imu_handler(self, imu_data):

        for key in imu_data.keys():
            self.get_logger().info(key)

        ros_msg = Imu()
        ros_msg.header.stamp = self.get_clock().now().to_msg()
        ros_msg.header.frame_id = "imu_link"
        # ros_msg.orientation.x = imu_data["QUATERNION"]["x"]
        # ros_msg.orientation.y = imu_data["QUATERNION"]["y"]
        # ros_msg.orientation.z = imu_data["QUATERNION"]["z"]
        # ros_msg.orientation.w = imu_data["QUATERNION"]["w"]
        # ros_msg.angular_velocity.x = imu_data["GYROSCOPE"]["x"]
        # ros_msg.angular_velocity.y = imu_data["GYROSCOPE"]["y"]
        # ros_msg.angular_velocity.z = imu_data["GYROSCOPE"]["z"]
        ros_msg.linear_acceleration.x = imu_data["ACCELEROMETER"]["X"] # NOTE: doesn't work

        # TODO: convert rpy to quaternion
        # NOTE: The following code is rubbish as the angle needs to be transformed first        # NOTE: The following code is rubbish as the angle needs to be transformed first
        ros_msg.orientation.x = imu_data["IMU"]["Roll"]
        ros_msg.orientation.y = imu_data["IMU"]["Pitch"]
        ros_msg.orientation.z = imu_data["IMU"]["Yaw"]

        self.imu_pub.publish(ros_msg)

    def encoder_handler(self, encoder_data):
        # self.lwheel_pub.publish(Int32(data=encoder_data["leftMotor"]["step_count"]))
        # self.lwheel_pub.publish(Int32(data=encoder_data["leftMotor"]["step_count"]))
        pass

    def listener_callback(self, msg):

        # calculate l and r speed from twist
        left_speed = lut_lookup(msg.linear.x - (msg.angular.z * self.wheel_base_in_m / 2))
        right_speed = lut_lookup(msg.linear.x + (msg.angular.z * self.wheel_base_in_m / 2))

        rvr.raw_motors(
            left_mode=RawMotorModesEnum.forward.value,
            left_duty_cycle=left_speed,  # Valid duty cycle range is 0-255
            right_mode=RawMotorModesEnum.forward.value,
            right_duty_cycle=right_speed  # Valid duty cycle range is 0-255
        )

    def wake_up_service_callback(self, _, response):
        # TODO: implement wake up service
        rvr.wake()
        time.sleep(2)
        self.get_logger().info("woke up")
        response.success = True
        return response

    def sleep_service_callback(self, _, response):
        # TODO: implement sleep service
        rvr.sleep()
        self.get_logger().info("sleeping")
        response.success = True
        return response

    def get_battery_service_callback(self, _, response):
        # TODO: implement sleep service
        response.message = "100"
        response.success = True
        return response


def main(args=None):
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
    try:
        main()
    finally:
        rvr.sensor_control.clear()
        time.sleep(0.5)
        rvr.close()
