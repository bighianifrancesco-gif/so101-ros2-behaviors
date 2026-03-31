#!/usr/bin/env python3

import math
import serial
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


ACC_SCALE = 16384.0

R_IMU_TO_ROBOT_DEFAULT = np.array([
    [9.99986964e-01, 4.92897828e-05, 5.10586733e-03],
    [0.00000000e+00, 9.99953408e-01, -9.65310750e-03],
    [5.10610524e-03, -9.65298166e-03, -9.99940372e-01]
], dtype=float)


def parse_imu_line(line: str):
    parts = line.strip().split(",")
    if len(parts) != 7:
        return None
    try:
        t_ms = int(parts[0])
        ax = int(parts[1])
        ay = int(parts[2])
        az = int(parts[3])
        gx = int(parts[4])
        gy = int(parts[5])
        gz = int(parts[6])
        return t_ms, ax, ay, az, gx, gy, gz
    except ValueError:
        return None


def gravity_to_roll_pitch(g_robot):
    gx, gy, gz = g_robot
    roll_deg = math.degrees(math.atan2(gy, -gz))
    pitch_deg = math.degrees(math.atan2(gx, math.sqrt(gy * gy + gz * gz)))
    return roll_deg, pitch_deg


class ImuSerialNode(Node):
    def __init__(self):
        super().__init__("imu_serial_node")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("alpha_acc", 0.18)

        self.port = self.get_parameter("port").get_parameter_value().string_value
        self.baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.alpha_acc = self.get_parameter("alpha_acc").get_parameter_value().double_value

        self.imu_pub = self.create_publisher(Imu, "/imu", 10)
        self.pitch_pub = self.create_publisher(Float64, "/imu_pitch", 10)
        self.roll_pub = self.create_publisher(Float64, "/imu_roll", 10)

        self.ser = None
        self.g_filt = np.array([0.0, 0.0, 1.0], dtype=float)
        self.r_imu_to_robot = R_IMU_TO_ROBOT_DEFAULT.copy()

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.001)
            self.get_logger().info(f"Opened IMU serial on {self.port} @ {self.baudrate}")
        except Exception as e:
            self.get_logger().error(f"Failed to open IMU serial {self.port}: {e}")
            raise

        self.timer = self.create_timer(0.005, self.timer_callback)

    def timer_callback(self):
        latest_line = None
        read_count = 0

        while self.ser.in_waiting > 0 and read_count < 200:
            line = self.ser.readline().decode("utf-8", errors="ignore").strip()
            if line:
                latest_line = line
            read_count += 1

        if latest_line is None:
            line = self.ser.readline().decode("utf-8", errors="ignore").strip()
            if line:
                latest_line = line

        if latest_line is None:
            return

        data = parse_imu_line(latest_line)
        if data is None:
            return

        t_ms, ax, ay, az, gx, gy, gz = data

        g_imu = np.array([ax, ay, az], dtype=float) / ACC_SCALE
        self.g_filt = (1.0 - self.alpha_acc) * self.g_filt + self.alpha_acc * g_imu
        g_robot = self.r_imu_to_robot @ self.g_filt

        roll_deg, pitch_deg = gravity_to_roll_pitch(g_robot)

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.orientation_covariance[0] = -1.0

        msg.angular_velocity.x = float(gx)
        msg.angular_velocity.y = float(gy)
        msg.angular_velocity.z = float(gz)

        msg.linear_acceleration.x = float(g_robot[0])
        msg.linear_acceleration.y = float(g_robot[1])
        msg.linear_acceleration.z = float(g_robot[2])

        self.imu_pub.publish(msg)

        pitch_msg = Float64()
        pitch_msg.data = float(pitch_deg)
        self.pitch_pub.publish(pitch_msg)

        roll_msg = Float64()
        roll_msg.data = float(roll_deg)
        self.roll_pub.publish(roll_msg)

    def destroy_node(self):
        try:
            if self.ser is not None:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()