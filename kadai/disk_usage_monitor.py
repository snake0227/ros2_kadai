#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Daichi Utsugi
# SPDX-License-Identifier: BSD-3-Clause

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
        self.target_dir = os.path.abspath(target_path)
        self.timer = self.create_timer(2.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        cpu_usage = psutil.cpu_percent()
        memory = psutil.virtual_memory()

        try:
            total, used, free = shutil.disk_usage(self.target_dir)
            drive_info = f"{free / (1024**3):.2f} GB / {total / (1024**3):.2f} GB"
        except Exception as e:
            drive_info = f"Disk Error: {e}"

        file_list = ""
        try:
            items = []
            if os.path.exists(self.target_dir):
                with os.scandir(self.target_dir) as entries:
                    for entry in entries:
                        try:
                            if entry.is_file():
                                items.append((entry.name, entry.stat().st_size))
                        except: continue
                items.sort(key=lambda x: x[1], reverse=True)
                for name, size in items[:5]:
                    file_list += f"  - {name}: {size / (1024**3):.4f} GB\n"
            else:
                file_list = "  Directory not found\n"
        except Exception as e:
            file_list = f"  Read Error: {e}\n"

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
    clean_args = rclpy.utilities.remove_ros_args(sys.argv)
    
    if len(clean_args) < 2:
        sys.stderr.write("ERROR: No target path provided.\n")
        sys.exit(1)

    target = clean_args[1]

    
    if not os.path.isdir(target):
        sys.stderr.write(f"ERROR: Directory '{target}' not found.\n")
        sys.exit(1)

    rclpy.init(args=args)
    
    try:
        node = SystemMonitor(target)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
    
        import traceback
        sys.stderr.write(f"CRITICAL ERROR: {e}\n")
        traceback.print_exc(file=sys.stderr)
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
