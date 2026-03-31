#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

from lerobot_kinematics.lerobot.feetech_arm import feetech_arm


JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


class ArmInterfaceNode(Node):
    def __init__(self):
        super().__init__("arm_interface_node")

        self.declare_parameter("port", "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0")
        self.declare_parameter("calibration_file", "/home/francesco/robotics/lerobot-kinematics/examples/main_follower.json")
        self.declare_parameter("publish_rate", 20.0)

        self.port = self.get_parameter("port").get_parameter_value().string_value
        self.calibration_file = self.get_parameter("calibration_file").get_parameter_value().string_value
        self.publish_rate = self.get_parameter("publish_rate").get_parameter_value().double_value

        self.joint_names = JOINT_NAMES

        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.cmd_sub = self.create_subscription(
            JointState,
            "/arm_cmd",
            self.arm_cmd_callback,
            10
        )

        self.get_logger().info(
            f"Connecting to arm on {self.port} with calibration {self.calibration_file}"
        )

        self.arm = feetech_arm(
            driver_port=self.port,
            calibration_file=self.calibration_file,
        )

        self.arm.arm_hardware.write("Torque_Enable", [1] * len(self.joint_names))

        timer_dt = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_dt, self.publish_joint_states)

        self.get_logger().info("Arm interface node started")

    def publish_joint_states(self):
        try:
            q = self.arm.feedback()

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(self.joint_names)
            msg.position = [float(x) for x in q]

            self.joint_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Failed to publish joint states: {e}")

    def arm_cmd_callback(self, msg: JointState):
        try:
            if not msg.name or not msg.position:
                self.get_logger().warning("Received empty /arm_cmd")
                return

            current_q = list(self.arm.feedback())
            name_to_index = {name: i for i, name in enumerate(self.joint_names)}

            for name, pos in zip(msg.name, msg.position):
                if name not in name_to_index:
                    self.get_logger().warning(f"Unknown joint name in /arm_cmd: {name}")
                    continue
                current_q[name_to_index[name]] = float(pos)

            self.arm.action(current_q)

        except Exception as e:
            self.get_logger().error(f"Failed to execute /arm_cmd: {e}")

    def destroy_node(self):
        try:
            if self.arm is not None:
                self.arm.disconnect()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArmInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
