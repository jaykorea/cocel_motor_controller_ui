# main.py
import sys
from PyQt5.QtWidgets import QApplication
import threading
import rclpy
from PyQt5.QtCore import QTimer
from motor_controller_node import MotorControllerNode
from motor_controller_ui import MotorControlUI

def run_ros_node(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    
    motor_control_node = MotorControllerNode()
    ui = MotorControlUI(motor_control_node)
    ui.show()

    # ROS 2 노드를 백그라운드 스레드에서 실행
    thread = threading.Thread(target=run_ros_node, args=(motor_control_node,))
    thread.start()

    exit_code = app.exec_()

    rclpy.shutdown()
    thread.join()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()
