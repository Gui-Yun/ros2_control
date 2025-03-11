#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import tty
import termios
import select

msg = """
机器人控制键盘:
---------------------------
移动命令:
   w    i    
a  s  d    
   k    l    

w/s : 前进/后退
a/d : 左转/右转
i/k : 慢速前进/后退
j/l : 慢速左转/右转
g   : 停止

q : 退出
"""

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher = self.create_publisher(String, 'cmd_key', 10)
        self.get_logger().info(msg)
        
    def getKey(self):
        """获取键盘输入"""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        """运行键盘控制循环"""
        self.settings = termios.tcgetattr(sys.stdin)
        
        while rclpy.ok():
            key = self.getKey()
            
            if key in ['w', 'a', 's', 'd', 'i', 'j', 'k', 'l', 'g']:
                # 发布按键命令
                msg = String()
                msg.data = key
                self.publisher.publish(msg)
                self.get_logger().info(f'发布按键: {key}')
            
            if key == 'q':
                break
                
        # 恢复终端设置
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    teleop = KeyboardTeleop()
    
    try:
        teleop.run()
    except Exception as e:
        print(e)
    finally:
        # 确保程序退出时恢复终端设置
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, teleop.settings)
        teleop.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 