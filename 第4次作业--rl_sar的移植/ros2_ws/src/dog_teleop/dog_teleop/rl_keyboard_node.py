import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


HELP_TEXT = """
RL 键盘控制节点
--------------------------------------------------
w / s : 前进 / 后退       改变 linear.x
a / d : 左移 / 右移       改变 linear.y
q / e : 左转 / 右转       改变 angular.z
空格   : 立即停止并归零
+ / - : 增大 / 减小按键步长
r     : 速度与倍率全部复位
h     : 显示帮助
Ctrl+C: 退出
--------------------------------------------------
"""


class RlKeyboardNode(Node):
    def __init__(self):
        super().__init__("rl_keyboard_node")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("publish_rate", 20.0)

        self.declare_parameter("vx_step", 0.10)
        self.declare_parameter("vy_step", 0.10)
        self.declare_parameter("yaw_step", 0.10)

        self.declare_parameter("max_vx", 0.50)
        self.declare_parameter("max_vy", 0.30)
        self.declare_parameter("max_yaw", 0.50)

        self.declare_parameter("speed_scale_min", 0.10)
        self.declare_parameter("speed_scale_max", 2.00)

        self.topic = str(self.get_parameter("cmd_vel_topic").value)
        self.publish_rate = max(
            1.0, float(self.get_parameter("publish_rate").value)
        )

        self.vx_step = float(self.get_parameter("vx_step").value)
        self.vy_step = float(self.get_parameter("vy_step").value)
        self.yaw_step = float(self.get_parameter("yaw_step").value)

        self.max_vx = float(self.get_parameter("max_vx").value)
        self.max_vy = float(self.get_parameter("max_vy").value)
        self.max_yaw = float(self.get_parameter("max_yaw").value)

        self.speed_scale_min = float(
            self.get_parameter("speed_scale_min").value
        )
        self.speed_scale_max = float(
            self.get_parameter("speed_scale_max").value
        )

        self.vx = 0.0
        self.vy = 0.0
        self.yaw = 0.0
        self.speed_scale = 1.0

        self.publisher = self.create_publisher(Twist, self.topic, 10)
        self.timer = self.create_timer(
            1.0 / self.publish_rate, self.publish_command
        )

        self.stdin_fd = sys.stdin.fileno()
        self.old_terminal_settings = None

        self.publish_command()
        self.get_logger().info(f"发布 /cmd_vel 到: {self.topic}")

    @staticmethod
    def clip(value, lower, upper):
        return max(lower, min(value, upper))

    def enable_keyboard(self):
        if not sys.stdin.isatty():
            raise RuntimeError(
                "必须在交互式终端中运行，例如 ros2 run dog_teleop rl_keyboard"
            )

        self.old_terminal_settings = termios.tcgetattr(self.stdin_fd)
        tty.setcbreak(self.stdin_fd)

    def restore_terminal(self):
        if self.old_terminal_settings is not None:
            termios.tcsetattr(
                self.stdin_fd,
                termios.TCSADRAIN,
                self.old_terminal_settings,
            )
            self.old_terminal_settings = None

    def publish_command(self):
        msg = Twist()
        msg.linear.x = self.vx
        msg.linear.y = self.vy
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.yaw
        self.publisher.publish(msg)

    def stop(self):
        self.vx = 0.0
        self.vy = 0.0
        self.yaw = 0.0
        self.publish_command()

    def print_status(self):
        self.get_logger().info(
            "vx={:.2f}, vy={:.2f}, yaw={:.2f}, scale={:.2f}".format(
                self.vx, self.vy, self.yaw, self.speed_scale
            )
        )

    def handle_key(self, key):
        if key == "\x03":  # Ctrl+C
            raise KeyboardInterrupt

        if key in ("w", "W"):
            self.vx = self.clip(
                self.vx + self.vx_step * self.speed_scale,
                -self.max_vx,
                self.max_vx,
            )
        elif key in ("s", "S"):
            self.vx = self.clip(
                self.vx - self.vx_step * self.speed_scale,
                -self.max_vx,
                self.max_vx,
            )
        elif key in ("a", "A"):
            self.vy = self.clip(
                self.vy + self.vy_step * self.speed_scale,
                -self.max_vy,
                self.max_vy,
            )
        elif key in ("d", "D"):
            self.vy = self.clip(
                self.vy - self.vy_step * self.speed_scale,
                -self.max_vy,
                self.max_vy,
            )
        elif key in ("q", "Q"):
            self.yaw = self.clip(
                self.yaw + self.yaw_step * self.speed_scale,
                -self.max_yaw,
                self.max_yaw,
            )
        elif key in ("e", "E"):
            self.yaw = self.clip(
                self.yaw - self.yaw_step * self.speed_scale,
                -self.max_yaw,
                self.max_yaw,
            )
        elif key == " ":
            self.stop()
            self.get_logger().info("停止：vx=0, vy=0, yaw=0")
            return
        elif key in ("+", "="):
            self.speed_scale = self.clip(
                self.speed_scale * 1.1,
                self.speed_scale_min,
                self.speed_scale_max,
            )
        elif key in ("-", "_"):
            self.speed_scale = self.clip(
                self.speed_scale / 1.1,
                self.speed_scale_min,
                self.speed_scale_max,
            )
        elif key in ("r", "R"):
            self.vx = 0.0
            self.vy = 0.0
            self.yaw = 0.0
            self.speed_scale = 1.0
        elif key in ("h", "H", "?"):
            print(HELP_TEXT)
            return
        else:
            return

        self.publish_command()
        self.print_status()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = RlKeyboardNode()
        node.enable_keyboard()
        print(HELP_TEXT)

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)

            ready, _, _ = select.select([sys.stdin], [], [], 0.0)
            if ready:
                key = sys.stdin.read(1)
                if key:
                    node.handle_key(key)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.stop()
            node.restore_terminal()
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()