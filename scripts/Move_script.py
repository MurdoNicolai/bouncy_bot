#!/usr/bin/python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage
import threading
import math

class Move(Node):


    def __init__(self):

        # initialize node
        super().__init__("move_node")
        self.arm_pos_sub = self.create_subscription(
            TFMessage,
            '/model/bouncy/pose',
            self.get_arms_callback,
            10
        )
        self.rob_pos_sub = self.create_subscription(
            Imu,
            '/imu',
            self.get_IMU_callback,
            10
        )

        self.turn_pub = self.create_publisher(msg_type=Float64,
                                                 topic="/turn_cmd",
                                                 qos_profile=1)

        self.arm_vel_pub1 = self.create_publisher(msg_type=Float64,
                                                 topic="/rot1_cmd",
                                                 qos_profile=1)

        self.arm_vel_pub2 = self.create_publisher(msg_type=Float64,
                                                 topic="/rot2_cmd",
                                                 qos_profile=1)

        self.jump_pub = self.create_publisher(msg_type=Float64,
                                                 topic="/jump_cmd",
                                                 qos_profile=1)

        # Start a thread to handle user input
        # self.class_thread = threading.Thread(target=self.process_input)
        # self.class_thread.daemon = True
        # self.class_thread.start()
        self.reset()
        print("ok")

    def reset(self):
        self.speed_arm1 = 0.0
        self.speed_arm2 = 0.0
        self.angle = 0.0
        self.jump_preped = False
        self.arm1_pos = 0.0
        self.arm2_pos = 0.0
        self.total_movement = 0.0
        self.max_movement = 0.0

        self.orx = 0.0
        self.ory = 0.0
        self.orz = 0.0
        self.orw = 0.0
        self.angleX = 0.0
        self.angleY = 0.0
        self.angleZ = 0.0

        self.jump()


    def return_state(self):
        return (self.speed_arm1, self.speed_arm2, self.angle,
                int(self.jump_preped), self.arm1_pos, self.arm2_pos,
                self.orx, self.ory, self.orz, self.orw,
                self.angleX, self.angleY, self.angleZ)

    def get_arms_callback(self, msg):

        try:
            for transform in msg.transforms:
                if transform.child_frame_id == 'bouncy/rotating_arm1':
                    quaternion = transform.transform.rotation
                    x_angle = math.atan2(2*(quaternion.w*quaternion.x + quaternion.y*quaternion.z),
                                         1 - 2*(quaternion.x**2 + quaternion.y**2))
                    self.arm1_pos = x_angle
                elif transform.child_frame_id == 'bouncy/rotating_arm2':
                    quaternion = transform.transform.rotation
                    x_angle = math.atan2(2*(quaternion.w*quaternion.x + quaternion.y*quaternion.z),
                                         1 - 2*(quaternion.x**2 + quaternion.y**2))
                    self.arm2_pos = x_angle

        except ValueError:
            print("Joint 'rotating_arm' not found in message")

    def get_IMU_callback(self, imu_msg):
        try:
            self.header_stamp = imu_msg.header.stamp
            self.frame_id = imu_msg.header.frame_id
            self.orientation = imu_msg.orientation
            self.orientation_covariance = imu_msg.orientation_covariance
            self.angular_velocity = imu_msg.angular_velocity
            self.angular_velocity_covariance = imu_msg.angular_velocity_covariance
            self.linear_acceleration = imu_msg.linear_acceleration
            self.linear_acceleration_covariance = imu_msg.linear_acceleration_covariance
            self.total_movement = self.magnitude(self.angular_velocity)

            self.orx = imu_msg.orientation.x
            self.ory = imu_msg.orientation.y
            self.orz = imu_msg.orientation.z
            self.orw = imu_msg.orientation.w
            self.angleX = imu_msg.angular_velocity.x
            self.angleY = imu_msg.angular_velocity.y
            self.angleZ =  imu_msg.angular_velocity.z


            self.max_movement_f(reset=False)
        except ValueError:
            print("Couldn't get current movement")
    def magnitude(self, vector):
        return math.sqrt(vector.x**2 + vector.y**2 + vector.z**2)

    def max_movement_f(self, reset=True):
        if reset:
            MM = self.max_movement
            self.max_movement = 0
            return MM
        if self.total_movement > self.max_movement:
            self.max_movement = self.total_movement

    def speed_apply(self):
        # apply speed valu to arms
        msg = Float64()
        msg.data = self.speed_arm1
        self.arm_vel_pub1.publish(msg)
        msg.data = self.speed_arm2
        self.arm_vel_pub2.publish(msg)

    def prep_jump(self):
        # prepare jump: arms go up and pull_engin goes down
        self.class_thread = threading.Thread(target=self.arm_up)
        self.class_thread.daemon = True
        self.class_thread.start()

        msg = Float64()
        msg.data = -2.0
        self.jump_pub.publish(msg)
        self.jump_preped = True

    def jump(self):
        self.class_thread = threading.Thread(target=self.arm_down)
        self.class_thread.daemon = True
        self.class_thread.start()
        msg = Float64()
        msg.data = 20.0
        self.jump_pub.publish(msg)
        self.jump_preped = False

    def cancel_jump(self):
        self.class_thread = threading.Thread(target=self.arm_down)
        self.class_thread.daemon = True
        self.class_thread.start()
        msg = Float64()
        msg.data = 2.0
        self.jump_pub.publish(msg)
        self.jump_preped = False

    def rotate(self):
        msg = Float64()
        msg.data = self.angle
        self.turn_pub.publish(msg)

    def turn(self):
        if self.speed_arm1 == 0:
            self.speed_arm1 = 0.3
        if self.speed_arm2 == 0:
            self.speed_arm2 = 0.3
        if (self.speed_arm1 * self.arm1_pos) > 0:
            self.arm_down(keep_speed=True)
        elif (self.speed_arm1 * self.arm1_pos) < 0:
            self.arm_up(keep_speed=True)
        msg = Float64()
        msg.data = self.angle
        self.turn_pub.publish(msg)

    def arm_down(self, keep_speed=False):
        Arm1_ok = False
        Arm2_ok = False
        while not Arm1_ok or not Arm2_ok:
            if self.arm1_pos < 3.0 and self.arm1_pos > -3.0:
                if not keep_speed:
                    self.speed_arm1 = 5.0
            elif self.arm1_pos < 3.1 and self.arm1_pos > -3.1:
                self.speed_arm1 = 0.5
            else:
                self.speed_arm1 = 0.0
                Arm1_ok = True
            if self.arm2_pos < 3.0 and self.arm2_pos > -3.0:
                if not keep_speed:
                    self.speed_arm2 = -5.0
            elif self.arm2_pos < 3.1 and self.arm2_pos > -3.1:
                self.speed_arm2 = -0.5
            else:
                self.speed_arm2 = 0.0
                Arm2_ok = True

            self.speed_apply()

    def arm_up(self, keep_speed=False):
        Arm1_ok = False
        Arm2_ok = False
        while not Arm1_ok or not Arm2_ok:
            if self.arm1_pos > 0.3 or self.arm1_pos < -0.3:
                if not keep_speed:
                    self.speed_arm1 = 10.0
            elif self.arm1_pos > 0.05 or self.arm1_pos < -0.05:
                self.speed_arm1 = 0.5
            else:
                self.speed_arm1 = 0.0
                Arm1_ok = True
            if self.arm2_pos > 0.3 or self.arm2_pos < -0.3:
                if not keep_speed:
                    self.speed_arm2 = -10.0
            elif self.arm2_pos > 0.05 or self.arm2_pos < -0.05:
                self.speed_arm2 = -0.5
            else:
                self.speed_arm2 = 0.0
                Arm2_ok = True
            self.speed_apply()

    def process_input(self):
        while True:
            user_input = input()
            if self.jump_preped:
                if user_input == " ":
                    self.jump()
                elif user_input == "b":
                    self.cancel_jump()
            else:
                if user_input == "z":
                    self.speed_arm1 += 0.2
                    self.speed_arm2 += 0.2
                elif user_input == "s":
                    self.speed_arm1 -= 0.2
                    self.speed_arm2 -= 0.2
                elif user_input == "q":
                    self.angle += 1
                    self.turn()
                elif user_input == "d":
                    self.angle -= 1
                    self.turn()
                elif user_input == "qq":
                    self.angle += 1
                    self.rotate()
                elif user_input == "dd":
                    self.angle -= 1
                    self.rotate()
                elif user_input == "a":
                    self.speed_arm1 += 0.2
                    self.speed_arm2 -= 0.2
                elif user_input == "e":
                    self.speed_arm1 -= 0.2
                    self.speed_arm2 += 0.2
                elif user_input == "b":
                    self.speed_arm1 = 0.0
                    self.speed_arm2 = 0.0
                elif user_input == " ":
                    self.prep_jump()
            self.speed_apply()

    def ai_input(self, user_input):
        if self.jump_preped:
            if user_input == " ":
                self.jump()
            elif user_input == "b":
                self.cancel_jump()
        else:
            if user_input == "z":
                self.speed_arm1 += 0.2
                self.speed_arm2 += 0.2
            elif user_input == "s":
                self.speed_arm1 -= 0.2
                self.speed_arm2 -= 0.2
            elif user_input == "q":
                self.angle += 1
                self.turn()
            elif user_input == "d":
                self.angle -= 1
                self.turn()
            elif user_input == "qq":
                self.angle += 1
                self.rotate()
            elif user_input == "dd":
                self.angle -= 1
                self.rotate()
            elif user_input == "a":
                self.speed_arm1 += 0.2
                self.speed_arm2 -= 0.2
            elif user_input == "e":
                self.speed_arm1 -= 0.2
                self.speed_arm2 += 0.2
            elif user_input == "b":
                self.speed_arm1 = 0.0
                self.speed_arm2 = 0.0
            elif user_input == " ":
                self.prep_jump()
        self.speed_apply()


def main(args=None):
    # initialize ROS2 communication
    rclpy.init(args=args)
    # initialize node
    move_node = Move()
    # spin the node
    rclpy.spin(move_node)
    # destroy the node
    move_node.destroy_node()
    # shutdown ROS2 communication
    rclpy.shutdown()

    return None

if __name__ == "__main__":
    main()

# End of Code
