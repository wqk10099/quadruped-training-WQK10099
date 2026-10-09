import rclpy
from rclpy.node import Node
import pygame
from my_interfaces.srv import SetMode

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')

        # --- 新东西 1：pygame 读手柄 ---
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            self.get_logger().error('没检测到手柄！')
        else:
            self.js = pygame.joystick.Joystick(0)
            self.js.init()
            self.get_logger().info(f'手柄已连接: {self.js.get_name()}')

        # --- 新东西 2：ROS 服务客户端 ---
        self.cli = self.create_client(SetMode, 'set_mode')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 set_mode 服务中...')

    def send_mode(self, mode: int):
        req = SetMode.Request()
        req.mode = mode
        future = self.cli.call_async(req)          # 异步调用，不阻塞
        future.add_done_callback(self.on_response)

    def on_response(self, future):
        res = future.result()                       # 拿到你定义的 response！
        self.get_logger().info(f'服务响应: success={res.success}, message={res.message}')

#主循环
BUTTON_MAP = {0: 1, 1: 2, 3: 0, 4: 3,6:4}   # 手柄按钮号 → 模式号（A→STAND, B→LIE, X→DAMPING, Y→MARCH,LB→WALK）

def main():
    rclpy.init()
    node = TeleopNode()
    try:
        while rclpy.ok():
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    mode = BUTTON_MAP.get(event.button)
                    if mode is not None:
                        node.get_logger().info(f'按下按钮 {event.button} → 模式 {mode}')
                        node.send_mode(mode)
            rclpy.spin_once(node, timeout_sec=0.01)   # 让 ROS 处理回调（比如服务响应）
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()