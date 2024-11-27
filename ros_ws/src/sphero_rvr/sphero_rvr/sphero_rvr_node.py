import os
import sys
import time

# ROS
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist, Quaternion, Vector3
from sensor_msgs.msg import Imu
from greeter_msgs.msg import EncoderTicks
import tf2_ros

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

fixed_vel = 0.2

# vel_lut = {
#       0: 0,
#      31: 0,
#      50: 0,
#      75: 0.05,
#      95: fixed_vel,
#     127: fixed_vel,
#     159: fixed_vel,
#     191: fixed_vel,
#     223: fixed_vel,
#     255: fixed_vel
# }

# def lut_lookup(vel_m_per_s, intervall=32):
#     if vel_m_per_s == 0:
#         return 0
#     lower_bound = (0, 0)    
#     upper_bound = (0, 0)

#     for key, value in vel_lut.items():
#         if vel_m_per_s < value:
#             upper_bound = (key, value)
#             break
#         lower_bound = (key, value)

#     # interpolate
#     if lower_bound[0] == upper_bound[0]:
#         return lower_bound[0]
#     else:
#         vel_per_pwm = (upper_bound[0] - lower_bound[0]) / (upper_bound[1] - lower_bound[1])
#         return lower_bound[0] + (vel_m_per_s - lower_bound[1]) / vel_per_pwm

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
        self.encoder_ticks = self.create_publisher(EncoderTicks, "encoder_ticks", 5)

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
            service=RvrStreamingServices.accelerometer,
            handler=self.acc_handler
        )
        rvr.sensor_control.add_sensor_data_handler(
            service=RvrStreamingServices.encoders,
            handler=self.encoder_handler
        )

        for k in rvr.sensor_control.supported_sensors:
            self.get_logger().info(k)

        rvr.sensor_control.start(interval=250)

        self.orientation = Quaternion()
        self.acc_lin = Vector3()
        self.imu_pub_timer = self.create_timer(1.0, self.publish_imu)


    def publish_imu(self):
        
        ros_msg = Imu()
        ros_msg.header.stamp = self.get_clock().now().to_msg()
        ros_msg.header.frame_id = "imu_link"

        ros_msg.orientation = self.orientation
        ros_msg.linear_acceleration = self.acc_lin

        self.imu_pub.publish(ros_msg)
        
    def acc_handler(self, acc_data):

        self.acc_lin.x = acc_data["Accelerometer"]["X"]
        self.acc_lin.y = acc_data["Accelerometer"]["Y"]
        self.acc_lin.z = acc_data["Accelerometer"]["Z"]

    def imu_handler(self, imu_data):

        with open("/home/dev/imu_log.txt", "w") as fobj:
            fobj.write(str(imu_data))

        for key in imu_data.keys():
            self.get_logger().info(key)

        # TODO: convert rpy to quaternion
        # NOTE: The following code is rubbish as the angle needs to be transformed first        # NOTE: The following code is rubbish as the angle needs to be transformed first
        # q = tf2_ros.quaternion()
        # q.setRPY(imu_data["IMU"]["Roll"],imu_data["IMU"]["Pitch"],imu_data["IMU"]["Yaw"])

        self.orientation.x = imu_data["IMU"]["Roll"]
        self.orientation.y = imu_data["IMU"]["Pitch"]
        self.orientation.z = imu_data["IMU"]["Yaw"]

    def map_value(self, ticks):
        if ticks >= 2**31:
            ticks = ticks - 2**32
            return ticks
        return ticks

    def encoder_handler(self, encoder_data):
        encoder_msg = EncoderTicks()

        encoders = encoder_data.get('Encoders')

        encoder_msg.left =  self.map_value(encoders.get('LeftTicks'))
        encoder_msg.right = self.map_value(encoders.get('RightTicks'))
        encoder_msg.header.stamp = self.get_clock().now().to_msg()
        
        self.encoder_ticks.publish(encoder_msg)

        
    def calc_pwm(self,vel_m_per_s):
        # 50->255
        # 0 ->  2
        d=(255-50) / 2
        if vel_m_per_s<0.01:
            return 0
        else:
            return min(50+d*vel_m_per_s, 255)
        
        
    def listener_callback(self, msg):
        # calculate l and r speed from twist
        left_speed = msg.linear.x - (msg.angular.z * self.wheel_base_in_m / 2)
        right_speed = msg.linear.x + (msg.angular.z * self.wheel_base_in_m / 2)

        if left_speed<0:
            left_mode=RawMotorModesEnum.reverse.value
        else:
            left_mode=RawMotorModesEnum.forward.value

        if right_speed<0:
            right_mode=RawMotorModesEnum.reverse.value
        else:
            right_mode=RawMotorModesEnum.forward.value

        left_duty_cycle=int(self.calc_pwm(abs(left_speed)))
        right_duty_cycle=int(self.calc_pwm(abs(right_speed)))
        self.get_logger().info(f"l: {left_duty_cycle:3}, r: {right_duty_cycle:3}")

        rvr.raw_motors(
            left_mode=left_mode,
            left_duty_cycle=left_duty_cycle,  # Valid duty cycle range is 0-255
            right_mode=right_mode,
            right_duty_cycle=right_duty_cycle # Valid duty cycle range is 0-255
        )

    def wake_up_service_callback(self, _, response):
        rvr.wake()
        time.sleep(2)
        self.get_logger().info("woke up")
        response.success = True
        return response

    def sleep_service_callback(self, _, response):
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
