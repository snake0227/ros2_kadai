#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Daichi Utsugi
# SPDX-License-Indentifier: BSD-3-Clause

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import sys

class SystemInfoSubscriber(Node):
    def __init__(self):
        super().__init__('system_info_sub')
        self.subscription = self.create_subscription(
            String, 'system_info', self.listener_callback, 10)
        
        self.last_time = time.time()
        self.msg_count = 0
        self.current_fps = 0.0

    def listener_callback(self, msg):
        self.msg_count += 1
        now = time.time()
        duration = now - self.last_time
        
        if duration >= 1.0:
            self.current_fps = self.msg_count / duration
            self.last_time = now
            self.msg_count = 0

        # \033[H\033[J は画面をクリアしてカーソルを左上に移動させる
        # flush=True を入れることで、溜め込まずにすぐ画面に出す
        sys.stdout.write("\033[H\033[J")
        output = (
            f"--- System Monitor Output (FPS: {self.current_fps:.1f}) ---\n"
            f"{msg.data}\n"
        )
        sys.stdout.write(output)
        sys.stdout.flush()

def main(args=None):
    rclpy.init(args=args)
    node = SystemInfoSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 終了時のエラーを防ぐため、安全にノードを閉じる
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
