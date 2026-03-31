#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState


JOINT_NAMES = [
    'shoulder_pan',
    'shoulder_lift',
    'elbow_flex',
    'wrist_flex',
    'wrist_roll',
    'gripper',
]

# -----------------------------
# Base poses
# -----------------------------
IDLE_POSE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Disappointed behavior, right side
ATTENTION_POSE_RIGHT = [0.0046, -1.2333, 0.4080, 1.2057, 0.0061, -0.0058]
NO_POSE_1_RIGHT = [-1.4113, -1.5539, 0.4602, 1.2057, 0.0061, -0.0058]
NO_POSE_2_RIGHT = [-1.8868, -1.5539, 0.4663, 1.2057, 0.0077, -0.0058]
NO_POSE_3_RIGHT = [-1.2149, -1.5432, 0.4694, 1.2057, 0.0077, -0.0058]

# Scared behavior
SCARED_POSE = [0.0307, -1.3944, 1.2364, -1.1551, 0.0061, -0.0058]

# Angry bite behavior, right side
BITE_POSE_1_RIGHT = [-1.4864, -1.3223, 0.4418, 1.2962, -0.0, 0.7398]
BITE_POSE_2_RIGHT = [-1.4742, -0.4157, -0.3697, 1.2962, -0.0, 0.7398]
BITE_POSE_3_RIGHT = [-1.4726, -0.3712, 0.3344, 0.2102, -0.0, -0.1456]
BITE_POSE_4_RIGHT = [-1.4742, -0.7655, -0.2301, 1.4788, -0.0, -0.1456]


def mirror_pose_left_right(pose):
    """
    Mirror left/right using shoulder_pan and wrist_roll sign flip.
    """
    mirrored = list(pose)
    mirrored[0] = -mirrored[0]   # shoulder_pan
    mirrored[4] = -mirrored[4]   # wrist_roll
    return mirrored


def lerp_pose(pose_a, pose_b, alpha):
    alpha = max(0.0, min(1.0, alpha))
    return [
        (1.0 - alpha) * float(a) + alpha * float(b)
        for a, b in zip(pose_a, pose_b)
    ]


def smoothstep(alpha):
    alpha = max(0.0, min(1.0, alpha))
    return alpha * alpha * (3.0 - 2.0 * alpha)


ATTENTION_POSE_LEFT = mirror_pose_left_right(ATTENTION_POSE_RIGHT)
NO_POSE_1_LEFT = mirror_pose_left_right(NO_POSE_1_RIGHT)
NO_POSE_2_LEFT = mirror_pose_left_right(NO_POSE_2_RIGHT)
NO_POSE_3_LEFT = mirror_pose_left_right(NO_POSE_3_RIGHT)

BITE_POSE_1_LEFT = mirror_pose_left_right(BITE_POSE_1_RIGHT)
BITE_POSE_2_LEFT = mirror_pose_left_right(BITE_POSE_2_RIGHT)
BITE_POSE_3_LEFT = mirror_pose_left_right(BITE_POSE_3_RIGHT)
BITE_POSE_4_LEFT = mirror_pose_left_right(BITE_POSE_4_RIGHT)


class BehaviorController(Node):
    def __init__(self):
        super().__init__('behavior_controller_node')

        # General parameters
        self.declare_parameter('mode', 'disappointed')
        self.declare_parameter('roll_trigger_deg', 1.5)
        self.declare_parameter('pitch_trigger_deg', 2.0)
        self.declare_parameter('refractory_sec', 2.5)
        self.declare_parameter('post_recover_deadzone_sec', 1.2)
        self.declare_parameter('roll_direction_gain', -1.0)
        self.declare_parameter('cmd_rate_hz', 30.0)

        # Disappointed timing
        self.declare_parameter('attention_time', 1.5)
        self.declare_parameter('no_time', 0.8)
        self.declare_parameter('recover_time', 2.0)

        # Scared timing
        self.declare_parameter('scared_out_time', 0.4)
        self.declare_parameter('scared_back_time', 5.0)

        # Angry bite timing
        self.declare_parameter('bite_1_time', 1.1)
        self.declare_parameter('bite_2_time', 0.3)
        self.declare_parameter('bite_3_time', 0.4)
        self.declare_parameter('bite_4_time', 0.4)
        self.declare_parameter('bite_back_time', 1.2)

        # Sensor state
        self.pitch = None
        self.roll = None
        self.prev_pitch = None
        self.prev_roll = None

        # Behavior state
        self.state = 'startup'
        self.state_t0 = self.get_clock().now()
        self.last_trigger = self.get_clock().now()
        self.push_side = 'right'

        # ROS
        self.create_subscription(Float64, '/imu_pitch', self.pitch_cb, 10)
        self.create_subscription(Float64, '/imu_roll', self.roll_cb, 10)
        self.pub = self.create_publisher(JointState, '/arm_cmd', 10)

        rate = float(self.get_parameter('cmd_rate_hz').value)
        self.timer = self.create_timer(1.0 / rate, self.update)

        self.get_logger().info('Behavior node ready')

    # -----------------------------

    def pitch_cb(self, msg):
        self.pitch = float(msg.data)

    def roll_cb(self, msg):
        self.roll = float(msg.data)

    def dt(self):
        return (self.get_clock().now() - self.state_t0).nanoseconds * 1e-9

    def set_state(self, state_name):
        self.state = state_name
        self.state_t0 = self.get_clock().now()
        self.get_logger().info(f'-> {state_name}')

    def publish(self, pose):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = [float(x) for x in pose]
        self.pub.publish(msg)

    def deadzone(self):
        self.last_trigger = self.get_clock().now()
        self.prev_pitch = self.pitch
        self.prev_roll = self.roll

    def interp(self, pose_a, pose_b, duration):
        if duration <= 0.0:
            self.publish(pose_b)
            return True
        alpha = smoothstep(self.dt() / duration)
        self.publish(lerp_pose(pose_a, pose_b, alpha))
        return self.dt() >= duration

    # -----------------------------
    # Pose selectors
    # -----------------------------

    def get_attention_pose(self):
        return ATTENTION_POSE_RIGHT if self.push_side == 'right' else ATTENTION_POSE_LEFT

    def get_no_pose_1(self):
        return NO_POSE_1_RIGHT if self.push_side == 'right' else NO_POSE_1_LEFT

    def get_no_pose_2(self):
        return NO_POSE_2_RIGHT if self.push_side == 'right' else NO_POSE_2_LEFT

    def get_no_pose_3(self):
        return NO_POSE_3_RIGHT if self.push_side == 'right' else NO_POSE_3_LEFT

    def get_bite_pose_1(self):
        return BITE_POSE_1_RIGHT if self.push_side == 'right' else BITE_POSE_1_LEFT

    def get_bite_pose_2(self):
        return BITE_POSE_2_RIGHT if self.push_side == 'right' else BITE_POSE_2_LEFT

    def get_bite_pose_3(self):
        return BITE_POSE_3_RIGHT if self.push_side == 'right' else BITE_POSE_3_LEFT

    def get_bite_pose_4(self):
        return BITE_POSE_4_RIGHT if self.push_side == 'right' else BITE_POSE_4_LEFT

    # -----------------------------
    # Push detection
    # -----------------------------

    def detect_push(self):
        if self.roll is None:
            return False

        if self.prev_roll is None:
            self.prev_roll = self.roll
            return False

        dr = self.roll - self.prev_roll
        self.prev_roll = self.roll

        roll_trigger_deg = float(self.get_parameter('roll_trigger_deg').value)
        refractory_sec = float(self.get_parameter('refractory_sec').value)
        post_recover_deadzone_sec = float(self.get_parameter('post_recover_deadzone_sec').value)
        roll_direction_gain = float(self.get_parameter('roll_direction_gain').value)

        if (self.get_clock().now() - self.last_trigger).nanoseconds * 1e-9 < max(
            refractory_sec, post_recover_deadzone_sec
        ):
            return False

        if abs(dr) < roll_trigger_deg:
            return False

        effective_dr = roll_direction_gain * dr

        if effective_dr < 0.0:
            self.push_side = 'right'
        else:
            self.push_side = 'left'

        self.last_trigger = self.get_clock().now()
        self.get_logger().info(
            f'push detected | dr={dr:.2f} | side={self.push_side}'
        )
        return True

    # -----------------------------
    # Main update
    # -----------------------------

    def update(self):
        mode = str(self.get_parameter('mode').value)

        attention_time = float(self.get_parameter('attention_time').value)
        no_time = float(self.get_parameter('no_time').value)
        recover_time = float(self.get_parameter('recover_time').value)

        scared_out_time = float(self.get_parameter('scared_out_time').value)
        scared_back_time = float(self.get_parameter('scared_back_time').value)

        bite_1_time = float(self.get_parameter('bite_1_time').value)
        bite_2_time = float(self.get_parameter('bite_2_time').value)
        bite_3_time = float(self.get_parameter('bite_3_time').value)
        bite_4_time = float(self.get_parameter('bite_4_time').value)
        bite_back_time = float(self.get_parameter('bite_back_time').value)

        A = self.get_attention_pose()
        N1 = self.get_no_pose_1()
        N2 = self.get_no_pose_2()
        N3 = self.get_no_pose_3()

        B1 = self.get_bite_pose_1()
        B2 = self.get_bite_pose_2()
        B3 = self.get_bite_pose_3()
        B4 = self.get_bite_pose_4()

        # Startup
        if self.state == 'startup':
            self.publish(IDLE_POSE)
            if self.dt() > 2.0:
                self.deadzone()
                self.set_state('idle')
            return

        # Idle
        if self.state == 'idle':
            self.publish(IDLE_POSE)
            if self.detect_push():
                if mode == 'disappointed':
                    self.set_state('att')
                elif mode == 'scared':
                    self.set_state('scared')
                elif mode == 'angry':
                    self.set_state('bite_1')
            return

        # -----------------------------
        # Disappointed
        # -----------------------------
        if self.state == 'att':
            if self.interp(IDLE_POSE, A, attention_time):
                self.set_state('n1')
            return

        if self.state == 'n1':
            if self.interp(A, N1, no_time):
                self.set_state('n2')
            return

        if self.state == 'n2':
            if self.interp(N1, N2, no_time):
                self.set_state('n3')
            return

        if self.state == 'n3':
            if self.interp(N2, N3, no_time):
                self.set_state('n2b')
            return

        if self.state == 'n2b':
            if self.interp(N3, N2, no_time):
                self.set_state('recover_disappointed')
            return

        if self.state == 'recover_disappointed':
            if self.interp(N2, IDLE_POSE, recover_time):
                self.deadzone()
                self.set_state('idle')
            return

        # -----------------------------
        # Scared
        # -----------------------------
        if self.state == 'scared':
            if self.interp(IDLE_POSE, SCARED_POSE, scared_out_time):
                self.set_state('scared_recover')
            return

        if self.state == 'scared_recover':
            if self.interp(SCARED_POSE, IDLE_POSE, scared_back_time):
                self.deadzone()
                self.set_state('idle')
            return

        # -----------------------------
        # Angry bite (lazy mode replacement)
        # -----------------------------
        if self.state == 'bite_1':
            if self.interp(IDLE_POSE, B1, bite_1_time):
                self.set_state('bite_2')
            return

        if self.state == 'bite_2':
            if self.interp(B1, B2, bite_2_time):
                self.set_state('bite_3')
            return

        if self.state == 'bite_3':
            if self.interp(B2, B3, bite_3_time):
                self.set_state('bite_4')
            return

        if self.state == 'bite_4':
            if self.interp(B3, B4, bite_4_time):
                self.set_state('bite_recover')
            return

        if self.state == 'bite_recover':
            if self.interp(B4, IDLE_POSE, bite_back_time):
                self.deadzone()
                self.set_state('idle')
            return


def main():
    rclpy.init()
    node = BehaviorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()