from PyQt5.QtWidgets import QMessageBox, QApplication, QWidget, QVBoxLayout, QHBoxLayout, QSlider, QLineEdit, QLabel, QPushButton, QComboBox, QCheckBox
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QIcon, QPixmap
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import os
import sys
import rclpy
import threading, time
from threading import Event
from std_msgs.msg import Float32MultiArray
from motor_controller_node import MotorControllerNode
from cocel_driver.msg import CocelDriverCmd
from cocel_driver.msg import MotorDriverCmd
from std_msgs.msg import Float32MultiArray

icon_path = '/home/cocel/cocel_ws/controller_ui/icon/cocel2.png'
if not os.path.exists(icon_path):
    print(f"Icon file does not exist: {icon_path}")

class ClickableComboBox(QComboBox):
    clicked = pyqtSignal()  # 클릭 이벤트 시그널 정의

    def mousePressEvent(self, event):
        self.clicked.emit()  # 클릭 시 clicked 시그널을 방출
        super().mousePressEvent(event)  # 부모 클래스의 mousePressEvent 호출

class MotorControlUI(QWidget):
    def __init__(self, node):
        super().__init__()
        self.resize(800, 200)  # 너비 1000px, 높이 400px로 설정
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
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle('Motor Controller UI')
        self.setWindowIcon(QIcon(icon_path))
        self.show()
        self.layout = QVBoxLayout(self)
        self.connect_checkbox = QCheckBox("Connect to Network", self)
        self.connect_checkbox.stateChanged.connect(self.handle_ros_connection)
        self.layout.addWidget(self.connect_checkbox)

        # 토픽 입력 창과 주파수 입력 창을 수평 레이아웃에 배치
        topic_frequency_pub_stop_layout = QHBoxLayout()
        self.pub_topic_input = QLineEdit(self)
        self.pub_topic_input.setPlaceholderText("/cocel_driver_node/cmd_topic") # 칸에 옅게 써놓는거 
        self.pub_topic_input.setText("/cocel_driver_node/cmd_topic") # 칸에 미리 써놓는거
        self.pub_topic_input.setMaximumWidth(500)
        topic_frequency_pub_stop_layout.addWidget(QLabel("Publish Topic"))
        topic_frequency_pub_stop_layout.addWidget(self.pub_topic_input)
        topic_frequency_pub_stop_layout.addSpacing(20)

        self.frequency_input = QLineEdit("100", self)
        # self.frequency_input.setMinimumSize(50,25)
        self.frequency_input.setMaximumWidth(40)
        self.frequency_input.setAlignment(Qt.AlignRight)
        topic_frequency_pub_stop_layout.addWidget(QLabel("Control Frequency (Hz)"))
        topic_frequency_pub_stop_layout.addWidget(self.frequency_input)
        topic_frequency_pub_stop_layout.addStretch()

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
        for i in range(1, 7):  # Assuming 6 motors for demonstration
            self.motor_controls.append(self.create_motor_layout(i))

        sub_topic_layout = QHBoxLayout()
        # subscirbe 토픽 레이블 생성
        sub_topic_layout.addWidget(QLabel("Subscribe Topic"))
        # Subscribe 토픽 선택 콤보박스 생성
        self.sub_topic_input = ClickableComboBox(self)
        self.sub_topic_input.clicked.connect(self.update_topic_list)
        self.sub_topic_input.currentIndexChanged.connect(self.on_topic_selected)
        self.sub_topic_input.setMinimumWidth(300)
        sub_topic_layout.addWidget(self.sub_topic_input)


        # 버튼 생성
        self.toggle_plot_button = QPushButton('Toggle Plot', self)
        self.toggle_plot_button.clicked.connect(self.toggle_plot_visibility)
        self.toggle_plot_button.setMinimumWidth(200)
        sub_topic_layout.addWidget(self.toggle_plot_button)
        sub_topic_layout.addStretch()
        # 이미지를 표시할 QLabel 생성
        image_path = '/home/cocel/cocel_ws/controller_ui/icon/cocel.png'
        pixmap = QPixmap(image_path)

        # 이미지 리사이즈
        desired_width = 304  # 원하는 너비
        desired_height = 35  # 원하는 높이
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
            motor_id_input.setText(str(2))
        if motor_id == 2:
            motor_id_input.setText(str(5))
        if motor_id == 3:
            motor_id_input.setText(str(6))
        if motor_id == 4:
            motor_id_input.setText(str(7))
        if motor_id == 5:
            motor_id_input.setText(str(9))
        if motor_id == 6:
            motor_id_input.setText(str(11))

        motor_id_input.setAlignment(Qt.AlignRight)
        motor_layout.addWidget(motor_id_input)
        motor_layout.addSpacing(15) # motor_id 입력칸 다음 오른쪽 스페이싱

        control_refs = {'id_input': motor_id_input}

        for control in ['Pos', 'Vel', 'Tau']:
            slider = QSlider(Qt.Horizontal, self)
            slider.setMinimum(-10000)
            slider.setMaximum(10000)
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

    def updateSliderValue(self, value):
        slider = self.sender()
        if hasattr(slider, 'value_label'):
            value = value / 1000.0  # Convert integer value back to decimal
            slider.value_label.setText(f'{value:.4f}')  # Show value with four decimal places

    
    def publish_commands(self):        
        if self.pub_topic_input.text() and self.ros_connected:
            topic_name = self.pub_topic_input.text()
            motor_commands = CocelDriverCmd()

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
                    motor_driver_cmd.tau_ff = float(tau)/1000.0

                    motor_commands.motor_cmd.append(motor_driver_cmd)

                except ValueError:
                    print("Invalid on publish_commands() function value")
                    QMessageBox.warning(self, "Invalid Input", "One or more inputs are invalid. Please check your inputs and try again. Stopped Publishing")
                    self.stop_publishing()
                    return

            # 메시지 발행
            try:
                self.node.publish_motor_commands(topic_name, motor_commands)
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

    def set_zero(self):
        # 모든 모터 컨트롤의 슬라이더 값을 0으로 설정하고, 레이블을 '0'으로 업데이트하는 메서드입니다.
        for motor_control in self.motor_controls:
            for control in ['Pos', 'Vel', 'Tau']:
                slider = motor_control[control]['slider']
                label = motor_control[control]['label']
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

    def update_topic_list(self):
        self.sub_topic_input.blockSignals(True)  # 시그널 차단
        if rclpy.ok() and self.ros_connected:
            self.sub_topic_input.clear()  # 기존 항목을 클리어
            self.sub_topic_input.addItem("Select Topic...")
            topics = self.node.get_topic_names_and_types()  # ROS2 노드에서 사용 가능한 토픽 목록을 가져옴
            for topic_name, topic_type in topics:
                self.sub_topic_input.addItem(topic_name)  # 콤보 박스에 토픽 추가
                self.topic_types[topic_name] = topic_type  # 토픽 타입 저장
        else: 
            self.sub_topic_input.clear()

        self.sub_topic_input.blockSignals(False)  # 시그널 차단 해제


    def on_topic_selected(self, index):
        selected_topic = self.sub_topic_input.currentText()
        selected_topic_type = self.topic_types.get(selected_topic, [])
        if selected_topic == "Select Topic...":
            return
        # 선택된 토픽의 타입 확인
        elif 'std_msgs/msg/Float32MultiArray' not in selected_topic_type:
            QMessageBox.critical(self, "Error", "Selected topic is not of type 'Float32MultiArray'.")
            return

        # 새 구독 설정
        try:
            self.change_subscription(selected_topic)
        except Exception as e:
            QMessageBox.critical(self, "Subscription Error", str(e))

    def change_subscription(self, topic_name):
        with self.subscription_lock:
            if self.subscription is not None:
                self.node.destroy_subscription(self.subscription)
                self.subscription = None

            # 구독을 파괴한 후, 새 구독을 생성하기 전에 rclpy.spin_once 호출을 일시적으로 중지
            # 또는 spin_once 호출을 관리하는 로직을 조정

            self.subscription = self.node.create_subscription(
                Float32MultiArray, topic_name, self.topic_callback, 10)
            print(f"Subscribed to {topic_name}")

    def topic_callback(self, msg):
        # msg.data는 Float32MultiArray 메시지의 float32[] data 필드입니다.
        # 이 예제에서는 msg.data가 [motor_id, pos, vel, motor_id, pos, vel, ...] 형태로 되어 있다고 가정합니다.
        # 실제 데이터 구조에 맞게 아래 코드를 조정해야 할 수 있습니다.
        self.data = msg.data
        self.update_plot()

    def update_plot(self):
        self.plot_figure.clear()  # 이전 그래프 클리어
        ax = self.plot_figure.add_subplot(111)  # 새로운 서브플롯 추가
    
        # 데이터 분리
        motor_ids = self.data[0::4]  # 모든 모터 ID
        pos_values = self.data[1::4]  # 위치 값
        vel_values = self.data[2::4]  # 속도 값
        torque_values = self.data[3::4]  # 토크 값

        #print(motor_ids)
        #print(pos_values)
        #print(vel_values)
        #print(torque_values)
    
        # x 축 값 설정
        x_values = range(len(motor_ids))

        # 그래프 그리기
        #ax.plot(motor_ids, pos_values, 'ro-', label='Position')  # 위치
        ax.plot(x_values, vel_values, 'go-', label='Velocity')  # 속도
        ax.plot(x_values, torque_values, 'bo-', label='Torque')  # 토크
        
        # x 축 레이블 설정
        ax.set_xticks(x_values)
        ax.set_xticklabels([f'Motor {int(id)}' for id in motor_ids])

        # 그래프 축 및 범례 설정
        ax.set_xlabel('Motor ID')
        ax.set_ylabel('Values')
        ax.set_title('Motor Status')
        ax.legend()

        # y 축 범위 설정
        ax.set_ylim(-35.0, 35.0)
    
        self.plot_canvas.draw()  # 캔버스 리프레시

    def toggle_plot_visibility(self):
        # 플롯 캔버스의 가시성을 토글합니다.
        self.plot_canvas.setVisible(not self.plot_canvas.isVisible())

        # 만약 플롯이 가시적이라면 창의 높이를 조정합니다.
        if self.plot_canvas.isVisible():
            self.resize(self.width(), 700)  # 플롯이 보이는 경우 창의 높이를 800px로 설정
        else:
            self.resize(self.width(), 200)  # 플롯이 숨겨진 경우 창의 높이를 400px로 설정

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
    
    motor_control_node = MotorControllerNode()
    ui = MotorControlUI(motor_control_node)
    ui.show()

    # 종료 이벤트 생성
    exit_event = Event()

    # 스레드에 종료 이벤트 전달
    ui_update_thread = threading.Thread(target=update_ui_periodically, args=(app, exit_event))
    ui_update_thread.start()

    ros_thread = threading.Thread(target=run_ros_node, args=(motor_control_node, exit_event))
    ros_thread.start()

    exit_code = app.exec_()

    # 종료 이벤트 설정하여 스레드 종료 요청
    exit_event.set()

    ros_thread.join()
    ui_update_thread.join()

    sys.exit(exit_code)
if __name__ == '__main__':
    main()