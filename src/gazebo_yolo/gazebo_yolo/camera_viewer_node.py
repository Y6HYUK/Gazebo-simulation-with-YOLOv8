import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image  # ROS2 이미지 메시지 타입
from cv_bridge import CvBridge  # ROS 이미지를 OpenCV 이미지로 변환
from PyQt5.QtWidgets import QApplication, QWidget, QHBoxLayout, QLabel, QVBoxLayout, QSizePolicy
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt, QTimer
import sys
import cv2
from ultralytics import YOLO

class CameraViewer(Node):
    def __init__(self, app):
        super().__init__('camera_viewer')
        self.app = app
        self.bridge = CvBridge()
        
        # YOLOv8 모델 로드 (최신 모델로 변경 가능)
        self.model = YOLO('yolov8n.pt')  # yolov8n.pt는 가장 작은 모델, 필요에 따라 변경
        
        # QoS 설정
        self.qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 카메라 토픽 설정 (SDF 파일에 정의된 토픽과 일치)
        self.camera_topics = {
            '/cctv_1/cctv_camera_1/image_raw': 0,
            '/cctv_2/cctv_camera_2/image_raw': 1
        }

        # 구독자 생성 (개별 콜백 사용)
        self.create_subscription(Image, '/cctv_1/cctv_camera_1/image_raw', self.image_callback_1, self.qos)
        self.create_subscription(Image, '/cctv_2/cctv_camera_2/image_raw', self.image_callback_2, self.qos)
        
        # UI 설정
        self.window = QWidget()
        self.main_layout = QVBoxLayout()  # 전체 레이아웃을 수직으로 변경
        self.window.setLayout(self.main_layout)

        self.camera_layout = QHBoxLayout()
        self.main_layout.addLayout(self.camera_layout)

        # 카메라 이미지를 표시할 라벨 생성 (2개로 제한)
        self.image_labels = []
        self.camera_labels = []
        for i in range(2):
            camera_box = QVBoxLayout()  # 각 카메라를 수직 박스로 그룹화

            # 카메라 설명 라벨 추가
            camera_label = QLabel(f'Camera {i+1}')
            camera_label.setAlignment(Qt.AlignCenter)
            camera_label.setStyleSheet("font-weight: bold; font-size: 16px;")
            self.camera_labels.append(camera_label)
            camera_box.addWidget(camera_label)

            # 이미지 표시 라벨
            label = QLabel()
            label.setAlignment(Qt.AlignCenter)
            label.setStyleSheet("border: 1px solid black; background-color: black;")
            label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            label.setMinimumSize(640, 480)  # 최소 크기 설정을 640x480으로 유지
            label.setScaledContents(True)
            self.image_labels.append(label)
            camera_box.addWidget(label)

            self.camera_layout.addLayout(camera_box)

        # 윈도우 크기 설정 (초기 창 크기를 1280x960으로 설정)
        self.window.resize(720, 480)
        self.window.setWindowTitle('Camera Viewer')
        self.window.show()

        # QTimer 설정 (이벤트 루프 통합)
        self.timer = QTimer()
        self.timer.timeout.connect(self.spin_once)
        self.timer.start(10)  # 10ms마다 spin_once 호출

    def spin_once(self):
        """ROS2 콜백을 처리하는 함수"""
        rclpy.spin_once(self, timeout_sec=0)

    def image_callback_1(self, msg):
        """첫 번째 카메라 이미지 콜백 함수"""
        self.display_image(msg, 0, 'Camera 1')

    def image_callback_2(self, msg):
        """두 번째 카메라 이미지 콜백 함수"""
        self.display_image(msg, 1, 'Camera 2')

    def display_image(self, msg, index, camera_name):
        """ROS 이미지 메시지를 받아서 Qt 라벨에 표시하고 YOLOv8으로 객체 탐지하는 함수"""
        try:
            # ROS 이미지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # YOLOv8 객체 탐지 적용
            results = self.model(cv_image)

            # 탐지 결과를 이미지에 그리기
            annotated_frame = results[0].plot(line_width=1, font_size=10)  # 탐지된 객체에 바운딩 박스와 라벨을 그림

            # OpenCV 이미지를 다시 RGB로 변환
            annotated_frame = cv2.cvtColor(annotated_frame, cv2.COLOR_BGR2RGB)

            h, w, ch = annotated_frame.shape
            bytes_per_line = ch * w
            # OpenCV 이미지를 Qt 이미지로 변환 (데이터를 bytes로 변환)
            qt_image = QImage(annotated_frame.data.tobytes(), w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_image)
            self.image_labels[index].setPixmap(pixmap)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    """메인 함수"""
    rclpy.init(args=args)  # ROS2 초기화
    app = QApplication(sys.argv)  # Qt 애플리케이션 생성
    viewer = CameraViewer(app)  # 카메라 뷰어 인스턴스 생성
    try:
        sys.exit(app.exec_())  # Qt 이벤트 루프 실행
    except KeyboardInterrupt:
        pass
    viewer.destroy_node()  # 노드 종료
    rclpy.shutdown()  # ROS2 종료

if __name__ == '__main__':
    main()
