#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import asyncio
import aiohttp
import json
from typing import List, Dict

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # ESP32配置
        self.board_ip = "192.168.4.1"
        self.base_url = f"http://{self.board_ip}/js"
        
        # 创建订阅者，接收键盘命令
        self.subscription = self.create_subscription(
            String,
            'cmd_key',
            self.key_callback,
            10)
            
        # 命令映射
        self.commands = self.create_command_mapping()
        self.get_logger().info('机器人控制节点已启动')
        
    def create_command_mapping(self) -> Dict[str, List[str]]:
        """创建键盘映射到JSON命令的字典"""
        return {
            'w': self.move_json(True, 280),   # 前进
            'd': self.move_json(False, 280),  # 右转
            's': self.move_json(True, -280),  # 后退
            'a': self.move_json(False, -280), # 左转
            'i': self.move_json(True, 50),    # 慢速前进
            'l': self.move_json(False, 50),   # 慢速右转
            'k': self.move_json(True, -50),   # 慢速后退
            'j': self.move_json(False, -50),  # 慢速左转
            'g': self.move_json(True, 0),     # 停止
        }

    def motor_command_json(self, motor_id: int, rpm: int, act: int = 3) -> str:
        """生成单个电机的JSON命令"""
        cmd = {
            "T": 10010,
            "id": motor_id,
            "cmd": rpm,
            "act": act
        }
        return json.dumps(cmd)

    def ctrl_rpm_side_json(self, side: str, rpm: int, act: int = 3) -> List[str]:
        """生成控制一侧电机的JSON命令列表"""
        commands = []
        if side == 'left':
            commands.append(self.motor_command_json(3, rpm, act))
            commands.append(self.motor_command_json(2, rpm, act))
        elif side == 'right':
            commands.append(self.motor_command_json(1, rpm, act))
            commands.append(self.motor_command_json(4, rpm, act))
        return commands

    def move_json(self, direction: bool, rpm: int, act: int = 3) -> List[str]:
        """根据运动方向生成所有电机的JSON命令列表"""
        commands = []
        if direction:
            commands.extend(self.ctrl_rpm_side_json('left', rpm, act))
            commands.extend(self.ctrl_rpm_side_json('right', -rpm, act))
        else:
            commands.extend(self.ctrl_rpm_side_json('left', rpm, act))
            commands.extend(self.ctrl_rpm_side_json('right', rpm, act))
        return commands

    async def send_json_command_async(self, session: aiohttp.ClientSession, json_cmd_str: str):
        """异步发送单个JSON命令"""
        params = {'json': json_cmd_str}
        try:
            async with session.get(self.base_url, params=params, timeout=5) as response:
                result = await response.text()
                self.get_logger().info(f'发送命令: {json_cmd_str}')
                self.get_logger().info(f'返回结果: {result}')
        except Exception as e:
            self.get_logger().error(f'发送失败: {str(e)}')

    async def send_commands_concurrent(self, json_cmd_list: List[str]):
        """并发发送多个JSON命令"""
        async with aiohttp.ClientSession() as session:
            tasks = [self.send_json_command_async(session, cmd) for cmd in json_cmd_list]
            await asyncio.gather(*tasks)

    def key_callback(self, msg: String):
        """处理接收到的键盘命令"""
        key = msg.data
        if key in self.commands:
            # 创建事件循环来发送命令
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self.send_commands_concurrent(self.commands[key]))
            loop.close()
            self.get_logger().info(f'处理按键: {key}')

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 