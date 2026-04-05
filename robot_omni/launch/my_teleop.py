#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import sys
import select
import termios
import tty

msg = """
========================================
ĐIỀU KHIỂN ROBOT MECANUM (Bắn liên tục 10Hz)
========================================
Cụm phím di chuyển:
   u    i    o
   j    k    l
   m    ,    .

i / , : Tiến / Lùi
j / l : Trượt Trái / Phải (Strafe)
u / o : Xoay Trái / Phải
k     : Dừng lại (Phanh gấp)

q / z : Tăng / Giảm tốc độ tịnh tiến 10%
w / x : Tăng / Giảm tốc độ xoay 10%

Bấm CTRL-C để thoát
"""

moveBindings = {
    'i': (1.0, 0.0, 0.0, 0.0),
    ',': (-1.0, 0.0, 0.0, 0.0),
    'j': (0.0, 1.0, 0.0, 0.0),
    'l': (0.0, -1.0, 0.0, 0.0),
    'u': (0.0, 0.0, 0.0, 1.0),
    'o': (0.0, 0.0, 0.0, -1.0),
}

speedBindings = {
    'q': (1.1, 1.0),
    'z': (0.9, 1.0),
    'w': (1.0, 1.1),
    'x': (1.0, 0.9),
}

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    # Giới hạn thời gian chờ phím là 0.1 giây (để tạo tần số 10Hz)
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class MecanumTeleop(Node):
    def __init__(self):
        super().__init__('mecanum_teleop_node')
        # Chú ý: Đổi tên topic cho phù hợp với controller của bạn
        self.publisher_ = self.create_publisher(TwistStamped, '/mobile_base_controller/reference', 100)
        
        self.speed = 0.5 # Tốc độ tịnh tiến (m/s)
        self.turn = 1.0  # Tốc độ xoay (rad/s)

    def publish_cmd(self, x, y, z, th):
        msg = TwistStamped()
        
        # Trong ROS 2 Jazzy, cập nhật timestamp chính xác giúp tránh lỗi đồng bộ
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint' # Hoặc 'base_link' tùy setup của bạn
        
        msg.twist.linear.x = x * self.speed
        msg.twist.linear.y = y * self.speed
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = th * self.turn
        
        self.publisher_.publish(msg)

def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    teleop_node = MecanumTeleop()

    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0

    print(msg)

    try:
        while rclpy.ok():
            key = getKey(settings)
            
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                teleop_node.speed = teleop_node.speed * speedBindings[key][0]
                teleop_node.turn = teleop_node.turn * speedBindings[key][1]
                print(f"Tốc độ hiện tại:\tDi chuyển {teleop_node.speed:.2f}\tXoay {teleop_node.turn:.2f}")
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            elif key == 'k':
                x = 0.0
                y = 0.0
                z = 0.0
                th = 0.0
            else:
                if (key == '\x03'): # Bấm Ctrl+C để thoát
                    break
                # Nếu không bấm gì, vẫn giữ nguyên x, y, th cũ và tiếp tục bắn

            # Bắn lệnh liên tục mỗi 0.1s
            teleop_node.publish_cmd(x, y, z, th)
            
            # Quét các tiến trình ROS 2 ngầm
            rclpy.spin_once(teleop_node, timeout_sec=0.0)

    except Exception as e:
        print(f"Có lỗi xảy ra: {e}")

    finally:
        # Stop robot trước khi thoát
        teleop_node.publish_cmd(0.0, 0.0, 0.0, 0.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        teleop_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()