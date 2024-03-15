import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from cocel_driver.msg import CocelDriverCmd
from cocel_driver.msg import MotorDriverCmd

class MotorControllerNode(Node):
    def __init__(self, topic_name='motor_commands'):
        super().__init__('motor_controller_node')
        self.topic_name = topic_name
        self.publisher_ = self.create_publisher(CocelDriverCmd, self.topic_name, 10)
        self.subscription = None  # 구독자 초기화
        self.custom_publishers = {}  # 변경된 부분: 속성 이름을 custom_publishers로 변경

    def publish_motor_commands(self, topic_name, msg):
        publisher = self.get_publisher(topic_name)
        publisher.publish(msg)

    def get_publisher(self, topic_name):
        # 이미 생성된 발행자가 있으면 재사용
        if topic_name not in self.custom_publishers:
            self.custom_publishers[topic_name] = self.create_publisher(CocelDriverCmd, topic_name, 10)
        return self.custom_publishers[topic_name]
    
    def get_available_topics(self):
        topic_names_and_types = self.get_topic_names_and_types()
        # 토픽 이름만 추출하여 반환
        return [name for name, types in topic_names_and_types]