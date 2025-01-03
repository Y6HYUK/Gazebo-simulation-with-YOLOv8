# Gazebo Simulation with YOLOv8
## Demo
[Click to watch the video](https://www.youtube.com/watch?v=_ROxLggRWa8)

![Screenshot from 2025-01-03 20-12-47](https://github.com/user-attachments/assets/dff7d288-9aa2-4c0e-b38b-612ec7779e2f)
![Screenshot from 2025-01-03 20-13-07](https://github.com/user-attachments/assets/53cc610f-6266-4f28-a8a2-57a713b6c932)

## Introduction
Gazebo Simulation with YOLOv8는 ROS2 Humble과 Gazebo를 사용하여 시뮬레이션 환경에서 실시간 CCTV 카메라 피드를 PyQt5 GUI로 시각화하고, YOLOv8을 통해 객체 탐지를 수행하는 프로젝트입니다.

## Features
- **실시간 카메라 피드**: Gazebo 시뮬레이션 내 여러 CCTV 카메라의 라이브 스트림 표시
- **객체 탐지**: YOLOv8을 사용하여 실시간으로 객체 탐지 및 바운딩 박스 표시
- **GUI**: PyQt5 기반의 직관적인 사용자 인터페이스

## Prerequisites
- **운영체제**: Ubuntu 22.04 LTS
- **ROS2 배포판**: Humble
- **Python 버전**: Python 3.10
- **필수 패키지**: PyQt5, OpenCV (opencv-python-headless), Ultralytics YOLOv8

## Installation

### 1. ROS2 Humble 설치
[ROS2 공식 설치 가이드](https://docs.ros.org/en/humble/Installation.html)를 참고하여 ROS2 Humble을 설치하세요.

### 2. 필수 패키지 설치
```bash
sudo apt update
sudo apt install -y python3-pip
pip3 install --user PyQt5 opencv-python-headless ultralytics
```

### 3. 레포지토리 클론 및 패키지 빌드
```bash
mkdir ~/ros2_ws
cd ~/ros2_ws
git clone https://github.com/Y6HYUK/Gazebo-simulation-with-YOLOv8.git
colcon build 
source install/setup.bash
```

## Usage

### 1. Gazebo 시뮬레이션 실행
```bash
gazebo /ros2_ws/src/gazebo_yolo/gazebo_yolo/worlds/custom_gas_station.world
```

### 2. 카메라 뷰어 노드 실행
```bash
ros2 run gazebo_yolo camera_viewer_node
```

## Dependencies
- **ROS2 Humble**
- **Gazebo**
- **Python 3.10**
- **PyQt5**
- **OpenCV**
- **Ultralytics YOLOv8**
- **cv_bridge**

## Project Structure
```
gazebo_yolo_ros2_camera_viewer/
├── gazebo_yolo/
│   ├── __init__.py
│   ├── camera_viewer_node.py
│   └── worlds/
│       └── custom_gas_station.world
├── resource/
│   └── gazebo_yolo
├── setup.py
├── package.xml
└── README.md
```


## License
This project is licensed under the [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0).

## Acknowledgements
- **ROS2**
- **Gazebo**
- **Ultralytics YOLOv8**
- **PyQt5**
- **OpenCV**

---

- 이 README는 프로젝트의 주요 내용을 간결하게 정리한 것으로, 필요에 따라 추가적인 정보를 포함하거나 수정하실 수 있습니다.
- This README is a concise summary of the highlights of the project, and you are welcome to include or modify additional information as needed.
