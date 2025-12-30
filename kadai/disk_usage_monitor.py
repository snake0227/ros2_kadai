#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Daichi Utsugi
# SPDX-License-Indentifier: BSD-3-Claus

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import shutil
import os
import sys
import psutil

class SystemMonitor(Node):
    def __init__(self, target_path):
        super().__init__('system_monitor')
        self.publisher_ = self.create_publisher(String, 'system_info', 10)
        self.target_drive = '/mnt/c'
        self.target_dir = target_path
        self.timer = self.create_timer(2.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        cpu_usage = psutil.cpu_percent()
        memory = psutil.virtual_memory()

        try:
            total, used, free = shutil.disk_usage(self.target_drive)
            drive_info = f"{free / (1024**3):.2f} GB / {total / (1024**3):.2f} GB"
        except:
            drive_info = "Disk Error"

        file_list = ""
        items = []
        try:
            if os.path.exists(self.target_dir):
                with os.scandir(self.target_dir) as entries:
                    for entry in entries:
                        if entry.is_file():
                            items.append((entry.name, entry.stat().st_size))
                items.sort(key=lambda x: x[1], reverse=True)
                for name, size in items[:5]:
                    file_list += f"  - {name}: {size / (1024**3):.4f} GB\n"
            else:
                file_list = "  Directory no longer exists\n"
        except:
            file_list = "  Read Error\n"

        info = (
            f"\n[CPU Usage]: {cpu_usage}%"
            f"\n[RAM Usage]: {memory.percent}% ({memory.used // (1024**2)}MB / {memory.total // (1024**2)}MB)"
            f"\n[Disk Free]: {drive_info}"
            f"\n--- Top Files in {self.target_dir} ---\n"
            f"{file_list}"
        )
        
        msg.data = info
        self.publisher_.publish(msg)

def main(args=None):
    # 1. 引数の解析（ROS2用引数を除外）
    import sys
    clean_args = rclpy.utilities.remove_ros_args(sys.argv)
    target = clean_args[1] if len(clean_args) > 1 else '/mnt/c'
    
    # 2. 【最優先】ディレクトリチェック
    # rclpy.initの前にチェックしてsys.exit(1)すれば、ROS2のエラーに巻き込まれません
    if not os.path.isdir(target):
        sys.stderr.write(f"ERROR: Directory '{target}' not found.\n")
        sys.exit(1)

    # 3. パスが正しい場合のみROS2を開始
    rclpy.init(args=args)
    node = SystemMonitor(target)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl+Cを受け取った時はここに来る。何もしなければ正常終了(0)へ向かう
        pass
    except Exception as e:
        # 想定外のエラーが起きた場合はエラー内容を表示して1で落とす
        sys.stderr.write(f"Unexpected error: {e}\n")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    finally:
        # 最後に必ずクリーンアップを行う
        # rclpy.ok() をチェックすることで、二重シャットダウンエラーを防ぐ
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
