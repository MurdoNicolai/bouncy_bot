#!/usr/bin/python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
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
        self.speed_arm1 = 0.0
        self.speed_arm2 = 0.0
        self.angle = 0.0
        self.jump_preped = False
        self.arm1_pos = 0.0
        self.arm2_pos = 0.0


        # Start a thread to handle user input
        self.input_thread = threading.Thread(target=self.process_input)
        self.input_thread.daemon = True
        self.input_thread.start()

        self.jump()

        # Spin the node
        rclpy.spin(self)

    def get_arms_callback(self, msg):

        try:
            for transform in msg.transforms:
                if transform.child_frame_id == 'bouncy/rotating_arm1':
                    quaternion = transform.transform.rotation
                    x_angle = math.atan2(2*(quaternion.w*quaternion.x + quaternion.y*quaternion.z),
                                         1 - 2*(quaternion.x**2 + quaternion.y**2))
                    self.arm1_pos = x_angle
            for transform in msg.transforms:
                if transform.child_frame_id == 'bouncy/rotating_arm2':
                    quaternion = transform.transform.rotation
                    x_angle = math.atan2(2*(quaternion.w*quaternion.x + quaternion.y*quaternion.z),
                                         1 - 2*(quaternion.x**2 + quaternion.y**2))
                    self.arm2_pos = x_angle
            print("1")
        except ValueError:
            print("Joint 'rotating_arm1' not found in message")

    def speed_apply(self):
        # apply speed valu to arms
        msg = Float64()
        msg.data = self.speed_arm1
        self.arm_vel_pub1.publish(msg)
        msg.data = self.speed_arm2
        self.arm_vel_pub2.publish(msg)


    def prep_jump(self):
        # prepare jump: arms go up and pull_engin goes down
        while self.arm1_pos > 0.1 or self.arm1_pos < -0.1 or self.arm2_pos > 0.1 or self.arm2_pos < -0.1:
            if self.arm1_pos > 0.1 or self.arm1_pos < -0.1:
                self.speed_arm1 = 5.0
            else:
                self.speed_arm1 = 0.0
            if self.arm2_pos > 0.1 or self.arm2_pos < -0.1:
                self.speed_arm2 = -5.0
            else:
                self.speed_arm2 = 0.0
            self.speed_apply()
            self.speed_apply()
        self.speed_arm1 = 0.0
        self.speed_arm2 = 0.0
        self.speed_apply()

        msg = Float64()
        msg.data = -1.0
        self.jump_pub.publish(msg)
        self.jump_preped = True

    def jump(self):
        while (self.arm1_pos < 3.0 and self.arm1_pos > -3.0) or (self.arm2_pos > 3.0 and self.arm2_pos < -3.0):
            if self.arm1_pos < 3.0 and self.arm1_pos > -3.0:
                self.speed_arm1 = 5.0
            else:
                self.speed_arm1 = 0.0
            if self.arm2_pos < 3.0 and self.arm2_pos > -3.0:
                self.speed_arm2 = -5.0
            else:
                self.speed_arm2 = 0.0
            print(self.arm2_pos)
            self.speed_apply()

        msg = Float64()
        msg.data = 20.0
        self.jump_pub.publish(msg)
        self.jump_preped = False

    def turn(self):
        msg = Float64()
        msg.data = self.angle
        print(msg)
        self.turn_pub.publish(msg)


    def process_input(self):
        while True:
            user_input = input()
            if user_input == "z":
                self.speed_arm1 += 0.2
                self.speed_arm2 += 0.2
            elif user_input == "s":
                self.speed_arm1 -= 0.2
                self.speed_arm2 -= 0.2
            elif user_input == "q":
                self.angle += 0.5
            elif user_input == "d":
                self.angle -= 0.5
            elif user_input == " ":
                if self.jump_preped:
                    self.jump()
                else:
                    self.prep_jump()
            self.speed_apply()
            self.turn()


def main(args=None):
    # initialize ROS2 communication
    rclpy.init(args=args)
    # initialize node
    dmove_node = Move()
    # spin the node
    rclpy.spin(dmove_node)
    # destroy the node
    dmove_node.destroy_node()
    # shutdown ROS2 communication
    rclpy.shutdown()

    return None

if __name__ == "__main__":
    main()

# End of Code
