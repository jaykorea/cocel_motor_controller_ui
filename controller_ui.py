# -*- coding: utf-8 -*- 
from PyQt5.QtWidgets import QMessageBox, QApplication, QWidget, QVBoxLayout, QHBoxLayout, QSlider, QLineEdit, QLabel, QPushButton, QComboBox, QCheckBox
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QIcon, QPixmap, QPalette, QColor
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import numpy as np
import os
import sys
import rclpy
import threading, time
from threading import Event
from std_msgs.msg import Float32MultiArray
from controller_node import ControllerNode
from cocel_driver.msg import CocelDriverCmd
from cocel_driver.msg import MotorDriverCmd
from cocel_driver.msg import CocelWheelCmd
from cocel_driver.msg import MotorDriverStatus
from cocel_driver.msg import WheelDriverStatus
from cocel_driver.msg import ImuStatus
from std_msgs.msg import Float32MultiArray

CWD_path = os.getcwd()
cocel3_icon_path = os.path.join(CWD_path, 'icon/cocel3.png')
cocel2_icon_path = os.path.join(CWD_path, 'icon/cocel2.png')
cocel_icon_path = os.path.join(CWD_path, 'icon/cocel.png')
flamingo_path = os.path.join(CWD_path, 'icon/flamingo.png')

if not os.path.exists(cocel2_icon_path) or not os.path.exists(cocel_icon_path):
    print(f"Icon file does not exist")

class ClickableComboBox(QComboBox):
    clicked = pyqtSignal()  # 클릭 이벤트 시그널 정의

    def mousePressEvent(self, event):
        self.clicked.emit()  # 클릭 시 clicked 시그널을 방출
        super().mousePressEvent(event)  # 부모 클래스의 mousePressEvent 호출

class FlamingoControlUI(QWidget):
    def __init__(self, node):
        super().__init__()
        #self.resize(800, 200)  # 너비 1000px, 높이 400px로 설정
        self.setFixedSize(1350,365)
        self.node = node
        self.subscription = None
        self.ros_connected = False  # ROS 연결 상태 추적 변수 추가
        self.is_publishing = False
        self.publishing_enabled = False
        self.pre_topic_name = None
        self.topic_types = {}  # 토픽 이름과 타입을 매핑할 딕셔너리
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.publish_commands)
        self.subscription_lock = threading.Lock()  # 구독 변경을 위한 락 초기화
        pal = QPalette()
        pal.setColor(QPalette.Background,QColor(252,252,252))
        self.setAutoFillBackground(True)
        self.setPalette(pal)
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle('Flamingo Controller UI')
        self.setWindowIcon(QIcon(flamingo_path))
        self.show()
        self.layout = QVBoxLayout(self)
        self.connect_checkbox = QCheckBox("Connect to Network", self)
        self.connect_checkbox.stateChanged.connect(self.handle_ros_connection)
        self.layout.addWidget(self.connect_checkbox)
        #self.setStyleSheet("background-image: url(" + flamingo_path + "); background-repeat: no-repeat; background-position: center;")

        # 토픽 입력 창과 주파수 입력 창을 수평 레이아웃에 배치
        topic_frequency_pub_stop_layout = QHBoxLayout()
        self.pub_motor_topic_input = QLineEdit(self)
        self.pub_motor_topic_input.setPlaceholderText("/cocel_driver_node/cmd_motor") # 칸에 옅게 써놓는거 
        self.pub_motor_topic_input.setText("/cocel_driver_node/cmd_motor") # 칸에 미리 써놓는거
        self.pub_motor_topic_input.setMaximumWidth(400)
        topic_frequency_pub_stop_layout.addWidget(QLabel("Pub Topic(motor)"))
        topic_frequency_pub_stop_layout.addWidget(self.pub_motor_topic_input)
        topic_frequency_pub_stop_layout.addSpacing(5)

        # 아래쪽에 새로운 토픽 입력 칸 추가
        self.pub_wheel_topic_input = QLineEdit(self)
        self.pub_wheel_topic_input.setPlaceholderText("/cocel_driver_node/cmd_wheel") # 칸에 옅게 써놓는거 
        self.pub_wheel_topic_input.setText("/cocel_driver_node/cmd_wheel") # 칸에 미리 써놓는거
        self.pub_wheel_topic_input.setMaximumWidth(400)
        topic_frequency_pub_stop_layout.addWidget(QLabel("Pub Topic(wheel)"))
        topic_frequency_pub_stop_layout.addWidget(self.pub_wheel_topic_input)
        topic_frequency_pub_stop_layout.addSpacing(5)

        self.frequency_input = QLineEdit("100", self)
        # self.frequency_input.setMinimumSize(50,25)
        self.frequency_input.setMaximumWidth(40)
        self.frequency_input.setAlignment(Qt.AlignRight)
        topic_frequency_pub_stop_layout.addWidget(QLabel("Control Frequency (Hz)"))
        topic_frequency_pub_stop_layout.addWidget(self.frequency_input)
        #topic_frequency_pub_stop_layout.addStretch()

        # 새로운 'Set Zero' 버튼을 만들고 controls_layout에 추가합니다.
        self.set_zero_button = QPushButton('Set Zero', self)
        self.set_zero_button.setMinimumWidth(100)
        # self.set_zero_button.setMinimumSize(50,25)
        # self.set_zero_button.setMaximumSize(150,25)
        self.set_zero_button.clicked.connect(self.set_zero)  # 클릭 시 set_zero 함수를 호출합니다.


        self.publish_button = QPushButton('Publish', self)
        self.publish_button.setMinimumWidth(140)
        #self.publish_button.setMinimumSize(50,25)
        #self.publish_button.setMaximumSize(150,25)
        self.publish_button.clicked.connect(self.start_publishing)

        self.stop_button = QPushButton('Stop', self)
        self.stop_button.setMinimumWidth(140)
        #self.stop_button.setMinimumSize(50,25)
        #self.stop_button.setMaximumSize(150,25)
        self.stop_button.clicked.connect(self.stop_publishing)

        topic_frequency_pub_stop_layout.addWidget(self.set_zero_button)
        topic_frequency_pub_stop_layout.addWidget(self.publish_button)
        topic_frequency_pub_stop_layout.addWidget(self.stop_button)
        self.layout.addLayout(topic_frequency_pub_stop_layout)

        #self.zero_button.clicked.connect(self.stop_publishing)

        self.motor_controls = []
        self.wheel_controls = []
        for i in range(1, 7):  # Assuming 6 motors for demonstration
            self.motor_controls.append(self.create_motor_layout(i))

        self.wheel_controls.append(self.create_wheel_layout(1, 1))
        self.wheel_controls.append(self.create_wheel_layout(1, 0))

        sub_topic_layout = QHBoxLayout()
        # subscirbe 토픽 레이블 생성
        sub_topic_layout.addWidget(QLabel("Sub Topic(Motor)"))
        # Subscribe 토픽 선택 콤보박스 생성
        self.sub_topic_motor_input = ClickableComboBox(self)
        self.sub_topic_motor_input.clicked.connect(self.update_motor_topic_list)
        self.sub_topic_motor_input.currentIndexChanged.connect(self.on_topic_motor_selected)
        self.sub_topic_motor_input.setMinimumWidth(175)
        sub_topic_layout.addWidget(self.sub_topic_motor_input)
        sub_topic_layout.addSpacing(10) # motor_id 레이블 오른쪽 스페이싱

        sub_topic_layout.addWidget(QLabel("Sub Topic(Wheel)"))
        # Subscribe 토픽 선택 콤보박스 생성
        self.sub_topic_wheel_input = ClickableComboBox(self)
        self.sub_topic_wheel_input.clicked.connect(self.update_wheel_topic_list)
        self.sub_topic_wheel_input.currentIndexChanged.connect(self.on_topic_wheel_selected)
        self.sub_topic_wheel_input.setMinimumWidth(175)
        sub_topic_layout.addWidget(self.sub_topic_wheel_input)
        sub_topic_layout.addSpacing(10) # motor_id 레이블 오른쪽 스페이싱

        sub_topic_layout.addWidget(QLabel("Sub Topic(Imu)"))
        # Subscribe 토픽 선택 콤보박스 생성
        self.sub_topic_imu_input = ClickableComboBox(self)
        self.sub_topic_imu_input.clicked.connect(self.update_imu_topic_list)
        self.sub_topic_imu_input.currentIndexChanged.connect(self.on_topic_imu_selected)
        self.sub_topic_imu_input.setMinimumWidth(175)
        sub_topic_layout.addWidget(self.sub_topic_imu_input)
        sub_topic_layout.addSpacing(10) # motor_id 레이블 오른쪽 스페이싱

        # 버튼 생성
        self.toggle_plot_button = QPushButton('Toggle Plot', self)
        self.toggle_plot_button.clicked.connect(self.toggle_plot_visibility)
        self.toggle_plot_button.setMinimumWidth(120)
        sub_topic_layout.addWidget(self.toggle_plot_button)
        sub_topic_layout.addStretch()
        # 이미지를 표시할 QLabel 생성
        image_path = cocel_icon_path
        pixmap = QPixmap(image_path)
        # 이미지 리사이즈
        desired_width = 265  # 원하는 너비
        desired_height = 30  # 원하는 높이
        scaled_pixmap = pixmap.scaled(desired_width, desired_height)
        image_label = QLabel(self)
        image_label.setPixmap(scaled_pixmap)
        image_label.setAlignment(Qt.AlignRight)
        sub_topic_layout.addWidget(image_label)

        self.layout.addLayout(sub_topic_layout)

        # Matplotlib Figure와 Canvas 초기화
        self.plot_figure = Figure()
        self.plot_canvas = FigureCanvas(self.plot_figure)
        self.plot_canvas.setVisible(False)
        self.plot_layout = QVBoxLayout()
        self.plot_layout.addWidget(self.plot_canvas)
        self.layout.addLayout(self.plot_layout)

        self.setLayout(self.layout)


        # 토픽 입력 창에서 Enter 키 이벤트 처리
        #self.topic_input.returnPressed.connect(self.update_topic_list)

    def create_motor_layout(self, motor_id):
        motor_layout = QHBoxLayout()
        self.layout.addLayout(motor_layout)
        motor_layout.addWidget(QLabel(f"Motor {motor_id}: ID"))
        motor_layout.addSpacing(1) # motor_id 레이블 오른쪽 스페이싱

        motor_id_input = QLineEdit(self)
        motor_id_input.setFixedWidth(30)
        if motor_id == 1:
            motor_id_input.setText(str(7))
        if motor_id == 2:
            motor_id_input.setText(str(11))
        if motor_id == 3:
            motor_id_input.setText(str(12))
        if motor_id == 4:
            motor_id_input.setText(str(6))
        if motor_id == 5:
            motor_id_input.setText(str(5))
        if motor_id == 6:
            motor_id_input.setText(str(3))

        motor_id_input.setAlignment(Qt.AlignRight)
        motor_layout.addWidget(motor_id_input)
        motor_layout.addSpacing(15) # motor_id 입력칸 다음 오른쪽 스페이싱

        control_refs = {'id_input': motor_id_input}

        for control in ['Pos', 'Vel', 'Tau']:
            slider = QSlider(Qt.Horizontal, self)
            slider.setMinimum(-20000)
            slider.setMaximum(20000)
            slider.setMinimumWidth(250)
            slider.valueChanged.connect(self.updateSliderValue)
            motor_layout.addWidget(QLabel(control))
            motor_layout.addWidget(slider)

            value_label = QLabel('0', self)
            value_label.setFixedWidth(55)  # 값 레이블에 고정 너비 설정
            motor_layout.addWidget(value_label)
            slider.value_label = value_label

            control_refs[control] = {'slider': slider, 'label': value_label}
            #motor_layout.addSpacing(5)

        motor_layout.addSpacing(1) # kp 입력칸 다음 오른쪽 스페이싱
        for param in ['Kp', 'Kd']:
            motor_layout.addWidget(QLabel(param))
            gainField = QLineEdit(self)
            gainField.setFixedWidth(30)
            gainField.setText(str(0) if param == 'Kp' else str(1))
            motor_layout.addWidget(gainField)
            gainField.setAlignment(Qt.AlignRight)
            motor_layout.addSpacing(10) # kp 입력칸 다음 오른쪽 스페이싱
            control_refs[param] = gainField
        
        return control_refs

    def create_wheel_layout(self, motor_id, is_left_wheel):
        motor_layout = QHBoxLayout()
        self.layout.addLayout(motor_layout)
        
        if is_left_wheel:
            motor_layout.addWidget(QLabel(f"Motor L: ID"))
        else:
            motor_layout.addWidget(QLabel(f"Motor R: ID"))
        
        motor_layout.addSpacing(1) # motor_id 레이블 오른쪽 스페이싱

        motor_id_input = QLineEdit(self)
        motor_id_input.setFixedWidth(30)
        motor_id_input.setText(str(1))  # 모두 같은 ID 값을 사용한다면, 여기서 설정하면 됩니다.
        motor_id_input.setAlignment(Qt.AlignRight)
        motor_layout.addWidget(motor_id_input)
        motor_layout.addSpacing(15) # motor_id 입력칸 다음 오른쪽 스페이싱

        control_refs = {'id_input': motor_id_input}

        for control in ['Pos', 'Vel', 'Tau']:
            slider = QSlider(Qt.Horizontal, self)
            slider.setMinimum(-20000)
            slider.setMaximum(20000)
            slider.setMinimumWidth(250)
            slider.valueChanged.connect(self.updateSliderValue)
            motor_layout.addWidget(QLabel(control))
            motor_layout.addWidget(slider)

            value_label = QLabel('0', self)
            value_label.setFixedWidth(55)  # 값 레이블에 고정 너비 설정
            motor_layout.addWidget(value_label)
            slider.value_label = value_label

            control_refs[control] = {'slider': slider, 'label': value_label}
            #motor_layout.addSpacing(5)

        motor_layout.addSpacing(1) # kp 입력칸 다음 오른쪽 스페이싱

        motor_layout.addWidget(QLabel('Control Mode'))
        controlmodeField = QLineEdit(self)
        controlmodeField.setFixedWidth(30)
        controlmodeField.setText(str(4))
        motor_layout.addWidget(controlmodeField)
        controlmodeField.setAlignment(Qt.AlignRight)
        motor_layout.addSpacing(10) # kp 입력칸 다음 오른쪽 스페이싱
        control_refs['control_mode'] = controlmodeField
        control_refs['is_left_wheel'] = is_left_wheel
        return control_refs

    def updateSliderValue(self, value):
        slider = self.sender()
        if hasattr(slider, 'value_label'):
            value = value / 1000.0  # Convert integer value back to decimal
            slider.value_label.setText(f'{value:.4f}')  # Show value with four decimal places

    def publish_commands(self):        
        if (self.pub_motor_topic_input.text() or self.pub_wheel_topic_input.text()) and self.ros_connected:
            topic_name = self.pub_motor_topic_input.text()
            topic_name2 = self.pub_wheel_topic_input.text()
            motor_commands = CocelDriverCmd()
            wheel_commands = CocelWheelCmd()

            for motor_control in self.motor_controls:
                # 사전에서 위젯을 직접 참조
                motor_id_input = motor_control['id_input']
                pos_slider = motor_control['Pos']['slider']
                vel_slider = motor_control['Vel']['slider']
                tau_slider = motor_control['Tau']['slider']
                kp_input = motor_control['Kp']
                kd_input = motor_control['Kd']

                # QLineEdit에서 텍스트 추출 및 QSlider에서 값(value) 추출
                try:
                    motor_id = int(motor_id_input.text())
                    pos = pos_slider.value()
                    vel = vel_slider.value()
                    tau = tau_slider.value()
                    kp = float(kp_input.text() if kp_input.text() else '0')
                    kd = float(kd_input.text() if kd_input.text() else '0')

                    motor_commands.motor_id_arr.append(motor_id)

                    motor_driver_cmd = MotorDriverCmd()
                    motor_driver_cmd.pos = float(pos)/1000.0
                    motor_driver_cmd.vel = float(vel)/1000.0
                    motor_driver_cmd.kp = float(kp)
                    motor_driver_cmd.kd = float(kd)
                    motor_driver_cmd.tau = float(tau)/1000.0

                    motor_commands.motor_cmd.append(motor_driver_cmd)

                except ValueError:
                    print("Invalid on publish_commands() function value")
                    QMessageBox.warning(self, "Invalid Input", "One or more inputs are invalid. Please check your inputs and try again. Stopped Publishing")
                    self.stop_publishing()
                    return

            for wheel_control in self.wheel_controls:
                # 사전에서 위젯을 직접 참조
                wheel_id_input = wheel_control['id_input']
                pos_slider = wheel_control['Pos']['slider']
                vel_slider = wheel_control['Vel']['slider']
                tau_slider = wheel_control['Tau']['slider']
                control_mode_input = wheel_control['control_mode']
                is_left = wheel_control['is_left_wheel']

                # QLineEdit에서 텍스트 추출 및 QSlider에서 값(value) 추출
                try:
                    wheel_id = int(wheel_id_input.text())
                    pos = pos_slider.value()
                    vel = vel_slider.value()
                    tau = tau_slider.value()
                    control_mode = int(control_mode_input.text())
                    wheel_commands.wheel_id = wheel_id
                    wheel_commands.control_mode = control_mode
                    if is_left == 1:
                        wheel_commands.wheel_cmd.pos_l = float(pos)/1000.0
                        wheel_commands.wheel_cmd.vel_l = float(vel)/1000.0
                        wheel_commands.wheel_cmd.tau_l = float(tau)/1000.0
                    else:
                        wheel_commands.wheel_cmd.pos_r = float(pos)/1000.0
                        wheel_commands.wheel_cmd.vel_r = float(vel)/1000.0
                        wheel_commands.wheel_cmd.tau_r = float(tau)/1000.0

                except ValueError:
                    print("Invalid on publish_commands() function value")
                    QMessageBox.warning(self, "Invalid Input", "One or more inputs are invalid. Please check your inputs and try again. Stopped Publishing")
                    self.stop_publishing()
                    return

            # 메시지 발행
            try:
                self.node.publish_motor_commands(topic_name, motor_commands)
                self.node.publish_wheel_commands(topic_name2, wheel_commands)
            except ValueError as e:
                print("Invalid topic name")
                QMessageBox.warning(self, "publish motor commands Failed", "ROS connection is required. Please check the ROS connection.", e)
            #print(f'Published commands: {motor_commands}\r\n')

    def start_publishing(self):
        if not self.ros_connected:
            QMessageBox.warning(self, "ROS Disconnected", "ROS connection is required. Please check the ROS connection.")
            return
        for motor_control in self.motor_controls:
            kp_input = motor_control['Kp']
            kd_input = motor_control['Kd']
            kp_input.setEnabled(False)
            kd_input.setEnabled(False)
        for wheel_control in self.wheel_controls:
            control_mode_input = wheel_control['control_mode']
            control_mode_input.setEnabled(False)
        self.pub_motor_topic_input.setEnabled(False)
        self.pub_wheel_topic_input.setEnabled(False)
        self.frequency_input.setEnabled(False) # 여긴 칸 비활성화 때린는 파트요
        if not self.is_publishing:
            try:
                frequency = float(self.frequency_input.text())
                interval_ms = 1000 / frequency
                self.timer.start(int(interval_ms))
                self.is_publishing = True
            except ValueError:
                print("Invalid frequency value")
                QMessageBox.warning(self, "Invalid Input", "One or more inputs are invalid. Please check your inputs and try again.")

    def stop_publishing(self):
        if self.is_publishing:
            self.timer.stop()
            self.is_publishing = False

            for motor_control in self.motor_controls:
                kp_input = motor_control['Kp']
                kd_input = motor_control['Kd']
                kp_input.setEnabled(True)
                kd_input.setEnabled(True)
            for wheel_control in self.wheel_controls:
                control_mode_input = wheel_control['control_mode']
                control_mode_input.setEnabled(True)
            self.pub_motor_topic_input.setEnabled(True)
            self.pub_wheel_topic_input.setEnabled(True)
            self.frequency_input.setEnabled(True) # 여긴 칸 활성화 때린는 파트요
    def set_zero(self):
        # 모든 모터 컨트롤의 슬라이더 값을 0으로 설정하고, 레이블을 '0'으로 업데이트하는 메서드.
        for motor_control in self.motor_controls:
            for control in ['Pos', 'Vel', 'Tau']:
                slider = motor_control[control]['slider']
                label = motor_control[control]['label']
                slider.setValue(0)  # 슬라이더 값을 0으로 설정
                label.setText('0')  # 레이블의 텍스트를 '0'으로 업데이트
        for wheel_control in self.wheel_controls:
            for control in ['Pos', 'Vel', 'Tau']:
                slider = wheel_control[control]['slider']
                label = wheel_control[control]['label']
                slider.setValue(0)  # 슬라이더 값을 0으로 설정
                label.setText('0')  # 레이블의 텍스트를 '0'으로 업데이트            


    def handle_ros_connection(self, state):
        if state == Qt.Checked:
            if not rclpy.ok():
                try:
                    rclpy.init(args=None)
                    self.ros_connected = True
                    print("Connected to ROS")
                except Exception as e:
                    print("Failed to connect to ROS:", e)
                    self.connect_checkbox.setChecked(False)
                    self.ros_connected = False
            self.ros_connected = True
            print("Connected to ROS")
        else:
            if rclpy.ok():
                #rclpy.shutdown()
                self.ros_connected = False
                print("Disconnected from ROS")

    def update_motor_topic_list(self):
        self.sub_topic_motor_input.blockSignals(True)  # 시그널 차단
        if rclpy.ok() and self.ros_connected:
            self.sub_topic_motor_input.clear()  # 기존 항목을 클리어
            self.sub_topic_motor_input.addItem("Select Topic...")
            topics = self.node.get_topic_names_and_types()  # ROS2 노드에서 사용 가능한 토픽 목록을 가져옴
            for topic_name, topic_type in topics:
                self.sub_topic_motor_input.addItem(topic_name)  # 콤보 박스에 토픽 추가
                self.topic_types[topic_name] = topic_type  # 토픽 타입 저장
        else: 
            self.sub_topic_motor_input.clear()

        self.sub_topic_motor_input.blockSignals(False)  # 시그널 차단 해제

    def update_wheel_topic_list(self):
        self.sub_topic_wheel_input.blockSignals(True)  # 시그널 차단
        if rclpy.ok() and self.ros_connected:
            self.sub_topic_wheel_input.clear()  # 기존 항목을 클리어
            self.sub_topic_wheel_input.addItem("Select Topic...")
            topics = self.node.get_topic_names_and_types()  # ROS2 노드에서 사용 가능한 토픽 목록을 가져옴
            for topic_name, topic_type in topics:
                self.sub_topic_wheel_input.addItem(topic_name)  # 콤보 박스에 토픽 추가
                self.topic_types[topic_name] = topic_type  # 토픽 타입 저장
        else: 
            self.sub_topic_wheel_input.clear()

        self.sub_topic_wheel_input.blockSignals(False)  # 시그널 차단 해제

    def update_imu_topic_list(self):
        self.sub_topic_imu_input.blockSignals(True)  # 시그널 차단
        if rclpy.ok() and self.ros_connected:
            self.sub_topic_imu_input.clear()  # 기존 항목을 클리어
            self.sub_topic_imu_input.addItem("Select Topic...")
            topics = self.node.get_topic_names_and_types()  # ROS2 노드에서 사용 가능한 토픽 목록을 가져옴
            for topic_name, topic_type in topics:
                self.sub_topic_imu_input.addItem(topic_name)  # 콤보 박스에 토픽 추가
                self.topic_types[topic_name] = topic_type  # 토픽 타입 저장
        else: 
            self.sub_topic_imu_input.clear()

        self.sub_topic_imu_input.blockSignals(False)  # 시그널 차단 해제

    def on_topic_motor_selected(self, index):
        selected_topic = self.sub_topic_motor_input.currentText()
        selected_topic_type = self.topic_types.get(selected_topic, [])
        if selected_topic == "Select Topic...":
            return
        # 선택된 토픽의 타입 확인
        elif 'cocel_driver/msg/MotorDriverStatus' not in selected_topic_type:
            QMessageBox.critical(self, "Error", "Selected topic is not of type 'MotorDriverStatus'.")
            return

        try:
            self.change_subscription(selected_topic, 0)
        except Exception as e:
            QMessageBox.critical(self, "Subscription Error", str(e))

    def on_topic_wheel_selected(self, index):
        selected_topic = self.sub_topic_wheel_input.currentText()
        selected_topic_type = self.topic_types.get(selected_topic, [])
        if selected_topic == "Select Topic...":
            return
        # 선택된 토픽의 타입 확인
        elif 'cocel_driver/msg/WheelDriverStatus' not in selected_topic_type:
            QMessageBox.critical(self, "Error", "Selected topic is not of type 'WheelDriverStatus'.")
            return

        try:
            self.change_subscription(selected_topic, 1)
        except Exception as e:
            QMessageBox.critical(self, "Subscription Error", str(e))

    def on_topic_imu_selected(self, index):
        selected_topic = self.sub_topic_imu_input.currentText()
        selected_topic_type = self.topic_types.get(selected_topic, [])
        if selected_topic == "Select Topic...":
            return
        # 선택된 토픽의 타입 확인
        elif 'cocel_driver/msg/ImuStatus' not in selected_topic_type:
            QMessageBox.critical(self, "Error", "Selected topic is not of type 'ImuStatus'.")
            return

        try:
            self.change_subscription(selected_topic, 2)
        except Exception as e:
            QMessageBox.critical(self, "Subscription Error", str(e))

    def change_subscription(self, topic_name, device):
        with self.subscription_lock:
            if self.subscription is not None:
                self.node.destroy_subscription(self.subscription)
                self.subscription = None

            if device == 0:
                self.subscription = self.node.create_subscription(
                MotorDriverStatus, topic_name, self.topic_callback_motor, 10)
                print(f"Subscribed to {topic_name}")
            elif device == 1:
                self.subscription = self.node.create_subscription(
                WheelDriverStatus, topic_name, self.topic_callback_wheel, 10)
                print(f"Subscribed to {topic_name}")
            elif device == 2:
                self.subscription = self.node.create_subscription(
                ImuStatus, topic_name, self.topic_callback_imu, 10)
                print(f"Subscribed to {topic_name}")

    def topic_callback_motor(self, msg):
        self.motor_ids = msg.motor_id_arr
        self.driver_status = msg.driver_status
        self.update_plot_motor()
        
    def topic_callback_wheel(self, msg):
        self.wheel_ids = [msg.wheel_id]  # 단일 휠 ID를 리스트로 변환
        self.wheel_status = [msg.wheel_status]  # 단일 휠 상태를 리스트로 변환
        self.update_plot_wheel()

    def topic_callback_imu(self, msg):
        self.orientation = msg.orientation
        self.angular_velocity = msg.angular_velocity
        self.linear_acceleration = msg.linear_acceleration
        self.euler_angle = msg.euler_angle
        self.update_plot_imu()

    def update_plot_motor(self):
        self.plot_figure.clear()  # 이전 그래프 클리어
        ax = self.plot_figure.add_subplot(111)  # 새로운 서브플롯 추가

        # 데이터 분리
        pos_values = [status.pos for status in self.driver_status]  # 위치 값
        vel_values = [status.vel for status in self.driver_status]  # 속도 값
        torque_values = [status.tau for status in self.driver_status]  # 토크 값

        # x 축 값 설정
        x_values = range(len(self.motor_ids))

        # 그래프 그리기
        ax.plot(x_values, vel_values, 'go-', linewidth=2, marker='o', markersize=5, label='Velocity')  # 속도
        ax.plot(x_values, torque_values, 'bo-', linewidth=2, marker='o', markersize=5, label='Torque')  # 토크
        
        # 각 모터별로 각 x 값에 해당하는 데이터 포인트에 대한 정보 표시
        for motor_id, vel_value, torque_value, x in zip(self.motor_ids, vel_values, torque_values, x_values):
            ax.text(x, -29, f'Velocity: {vel_value:.2f}\nTorque: {torque_value:.2f}',
                    fontsize=8, ha='center', va='bottom')

        # x 축 레이블 설정
        ax.set_xticks(x_values)
        ax.set_xticklabels([f'Motor {int(id)}' for id in self.motor_ids])

        # 그래프 축 및 범례 설정
        ax.set_xlabel('Motor ID')
        ax.set_ylabel('Values')
        ax.set_title('Motor Status').set_fontsize('medium')
        ax.legend()

        # y 축 범위 및 레이블 설정
        ax.set_yticks([i for i in range(-30, 31, 3)])  # 시작 값, 끝 값, 간격
        ax.set_yticklabels(ax.get_yticks(), fontsize=9)  # 폰트 크기 지정

        self.plot_canvas.draw()  # 캔버스 리프레시

    def update_plot_wheel(self):
        self.plot_figure.clear()  # 이전 그래프 클리어
        ax = self.plot_figure.add_subplot(111)  # 새로운 서브플롯 추가

        # 데이터 분리
        pos_values_l = [status.pos_l for status in self.wheel_status]  # 왼쪽 위치 값
        pos_values_r = [status.pos_r for status in self.wheel_status]  # 오른쪽 위치 값
        vel_values_l = [status.vel_l for status in self.wheel_status]  # 왼쪽 속도 값
        vel_values_r = [status.vel_r for status in self.wheel_status]  # 오른쪽 속도 값
        torque_values_l = [status.tau_l for status in self.wheel_status]  # 왼쪽 토크 값
        torque_values_r = [status.tau_r for status in self.wheel_status]  # 오른쪽 토크 값

        # x 축 값 설정
        x_values = range(len(self.wheel_ids))

        # 그래프 그리기
        ax.plot(x_values, vel_values_l, 'go-', linewidth=2, marker='o', markersize=5, label='Velocity Left')  # 왼쪽 속도
        ax.plot(x_values, vel_values_r, 'ro-', linewidth=2, marker='o', markersize=5, label='Velocity Right')  # 오른쪽 속도
        ax.plot(x_values, torque_values_l, 'bo-', linewidth=2, marker='o', markersize=5, label='Torque Left')  # 왼쪽 토크
        ax.plot(x_values, torque_values_r, 'yo-', linewidth=2, marker='o', markersize=5, label='Torque Right')  # 오른쪽 토크

        # 각 휠별로 각 x 값에 해당하는 데이터 포인트에 대한 정보 표시
        for wheel_id, vel_l, vel_r, tau_l, tau_r, x in zip(self.wheel_ids, vel_values_l, vel_values_r, torque_values_l, torque_values_r, x_values):
            ax.text(x, -29, f'Vel L: {vel_l:.2f}\nTor L: {tau_l:.2f}\nVel R: {vel_r:.2f}\nTor R: {tau_r:.2f}',
                    fontsize=8, ha='center', va='bottom')

        # x 축 레이블 설정
        ax.set_xticks(x_values)
        ax.set_xticklabels([f'Wheel {int(id)}' for id in self.wheel_ids])

        # 그래프 축 및 범례 설정
        ax.set_xlabel('Wheel ID')
        ax.set_ylabel('Values')
        ax.set_title('Wheel Status').set_fontsize('medium')
        ax.legend()

        # y 축 범위 및 레이블 설정
        ax.set_yticks([i for i in range(-30, 31, 3)])  # 시작 값, 끝 값, 간격
        ax.set_yticklabels(ax.get_yticks(), fontsize=9)  # 폰트 크기 지정

        self.plot_canvas.draw()  # 캔버스 리프레시

    # def update_plot_wheel(self):
    #     self.plot_figure.clear()  # Clear previous graph

    #     # Define the axes for position, velocity, and torque
    #     ax1 = self.plot_figure.add_subplot(311)  # Position graph
    #     ax2 = self.plot_figure.add_subplot(312)  # Velocity graph
    #     ax3 = self.plot_figure.add_subplot(313)  # Torque graph

    #     # Check the number of data points to plot
    #     num_data_points = len(self.wheel_status)
    #     x_labels = ['Left', 'Right'] * num_data_points  # Repeat 'Left', 'Right' for each data point

    #     # Ensure the list has entries for each 'Left' and 'Right' label
    #     pos_values_l = [status.pos_l for status in self.wheel_status if hasattr(status, 'pos_l')]
    #     pos_values_r = [status.pos_r for status in self.wheel_status if hasattr(status, 'pos_r')]
    #     vel_values_l = [status.vel_l for status in self.wheel_status if hasattr(status, 'vel_l')]
    #     vel_values_r = [status.vel_r for status in self.wheel_status if hasattr(status, 'vel_r')]
    #     torque_values_l = [status.tau_l for status in self.wheel_status if hasattr(status, 'tau_l')]
    #     torque_values_r = [status.tau_r for status in self.wheel_status if hasattr(status, 'tau_r')]

    #     # Plotting each graph with checks to avoid errors
    #     if len(pos_values_l) == num_data_points and len(pos_values_r) == num_data_points:
    #         ax1.plot(x_labels[0::2], pos_values_l, 'ro-', linewidth=2, markersize=5, label='Position Left')
    #         ax1.plot(x_labels[1::2], pos_values_r, 'bo-', linewidth=2, markersize=5, label='Position Right')

    #     if len(vel_values_l) == num_data_points and len(vel_values_r) == num_data_points:
    #         ax2.plot(x_labels[0::2], vel_values_l, 'ro-', linewidth=2, markersize=5, label='Velocity Left')
    #         ax2.plot(x_labels[1::2], vel_values_r, 'bo-', linewidth=2, markersize=5, label='Velocity Right')

    #     if len(torque_values_l) == num_data_points and len(torque_values_r) == num_data_points:
    #         ax3.plot(x_labels[0::2], torque_values_l, 'ro-', linewidth=2, markersize=5, label='Torque Left')
    #         ax3.plot(x_labels[1::2], torque_values_r, 'bo-', linewidth=2, markersize=5, label='Torque Right')

    #     # Set titles and labels
    #     ax1.set_title('Wheel Position')
    #     ax1.set_ylabel('Position')
    #     ax1.legend()

    #     ax2.set_title('Wheel Velocity')
    #     ax2.set_ylabel('Velocity')
    #     ax2.legend()

    #     ax3.set_title('Wheel Torque')
    #     ax3.set_ylabel('Torque')
    #     ax3.legend()

    #     # Adjust layout and refresh canvas
    #     self.plot_figure.tight_layout()
    #     self.plot_canvas.draw()  # Refresh canvas

    def update_plot_imu(self):
        self.plot_figure.clear()  # 이전 그래프 클리어
        ax1 = self.plot_figure.add_subplot(221)  # 오리엔테이션 그래프
        ax2 = self.plot_figure.add_subplot(222)  # 각속도 그래프
        ax3 = self.plot_figure.add_subplot(223)  # 선형 가속도 그래프
        ax4 = self.plot_figure.add_subplot(224)  # 오일러 각 그래프

        # 각 데이터 추출 및 그래프 표시
        # 오리엔테이션 (쿼터니언)
        ax1.bar(['x', 'y', 'z', 'w'], [self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w])
        ax1.set_title('Orientation (Quaternion)')
        
        # 각속도
        ax2.plot(['x', 'y', 'z'], [self.angular_velocity.x, self.angular_velocity.y, self.angular_velocity.z], marker='o')
        ax2.set_title('Angular Velocity')

        # 선형 가속도
        ax3.plot(['x', 'y', 'z'], [self.linear_acceleration.x, self.linear_acceleration.y, self.linear_acceleration.z], marker='o')
        ax3.set_title('Linear Acceleration')

        # 오일러 각
        ax4.plot(['roll', 'pitch', 'yaw'], [self.euler_angle.x, self.euler_angle.y, self.euler_angle.z], marker='o')
        ax4.set_title('Euler Angles')

        # 전체 그래프에 대한 레이아웃 조정
        self.plot_figure.tight_layout()

        self.plot_canvas.draw()  # 캔버스 리프레시

    def toggle_plot_visibility(self):
        # 플롯 캔버스의 가시성을 토글합니다.
        self.plot_canvas.setVisible(not self.plot_canvas.isVisible())

        # 만약 플롯이 가시적이라면 창의 높이를 조정합니다.
        if self.plot_canvas.isVisible():
            self.setFixedSize(self.width(),755)
        else:
            self.setFixedSize(self.width(),365)

    def closeEvent(self, event):
        if self.is_publishing:
            reply = QMessageBox.question(self, 'Publishing in progress',
                                         "Publishing is still in progress. Are you sure you want to stop publishing and exit?",
                                         QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

            if reply == QMessageBox.Yes:
                self.stop_publishing()  # 발행을 중단합니다.
                rclpy.shutdown()
                event.accept()  # 프로그램 종료 이벤트를 수락합니다.
            else:
                event.ignore()  # 프로그램 종료 이벤트를 무시합니다.
        else:
            event.accept()  # 발행이 진행 중이지 않으면 프로그램 종료 이벤트를 수락합니다.
        
# def safely_spin_once(node):
#     try:
#         rclpy.spin_once(node, timeout_sec=0.1)
#     except Exception as e:
#         print(f"Error during spin_once: {e}")

stylesheet = """
    FlamingoControlUI {{
        background-image: url("{0}");
        background-repeat: no-repeat; 
        background-position: center;
    }}
""".format(flamingo_path)


def run_ros_node(node, event):
    while not event.is_set():
        rclpy.spin_once(node, timeout_sec=0.1)
    rclpy.shutdown()

def update_ui_periodically(app, event):
    while not event.is_set():
        app.processEvents()
        time.sleep(0.01)  # 이벤트 루프가 너무 빠르게 돌지 않도록 약간의 지연을 추가

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    app.setStyleSheet(stylesheet)
    
    flamingo_control_node = ControllerNode()
    ui = FlamingoControlUI(flamingo_control_node)
    ui.show()

    # 종료 이벤트 생성
    exit_event = Event()

    # 스레드에 종료 이벤트 전달
    ui_update_thread = threading.Thread(target=update_ui_periodically, args=(app, exit_event))
    ui_update_thread.start()

    ros_thread = threading.Thread(target=run_ros_node, args=(flamingo_control_node, exit_event))
    ros_thread.start()

    exit_code = app.exec_()

    # 종료 이벤트 설정하여 스레드 종료 요청
    exit_event.set()

    ui_update_thread.join()
    ros_thread.join()

    sys.exit(exit_code)
if __name__ == '__main__':
    main()
