# PUMA 3-DOF Robot System (ROS2 + Web UI)

이 프로젝트는 ROS 2 (Humble)를 기반으로 구성된 3자유도 PUMA 타입 로봇 시스템입니다. URDF 모델과 RViz 시각화, 웹 기반 제어 인터페이스를 제공하며, 실시간 관절 제어 및 로깅 기능을 포함합니다.

---

## 📂 프로젝트 구조

```bash
puma_robot/
├── CMakeLists.txt
├── package.xml
├── setup.py
├── urdf/
│   └── puma_3dof.urdf.xacro        # 로봇 URDF 정의 (xacro)
├── launch/
│   └── display.launch.py           # URDF 로드 및 RViz 실행용 런치 파일
├── scripts/
│   ├── puma_controller.py          # ROS2 제어 노드
│   ├── joint_state_logger.py       # 관절 상태 로그 저장 스크립트
│   ├── ros2_web_bridge.py          # 웹-ROS2 통신 브릿지
│   ├── test.py                     # 테스트 스크립트
│   ├── test.html                   # 웹 테스트 페이지
│   ├── puma_web_ui.html            # 로봇 제어용 웹 UI
│   └── logger_web_ui.html          # 로깅 상태 확인용 UI
├── rviz/
│   └── puma_config.rviz            # RViz 설정 파일
```

---

## 🛠️ 실행 환경

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10 이상
- xacro, joint_state_publisher, robot_state_publisher
- 웹 브라우저 (Chrome 권장)

---

## 🧩 실행 방법

### 1. ROS 2 패키지 빌드

```bash
cd ~/ros2_ws
colcon build --packages-select puma_robot
source install/setup.bash
```

### 2. RViz로 로봇 모델 확인

```bash
ros2 launch puma_robot display.launch.py
```

### 3. 웹 UI 사용 (HTML 직접 실행)

```bash
# puma_web_ui.html 또는 logger_web_ui.html 파일을 브라우저에서 열기
open scripts/puma_web_ui.html
```

> 또는 `ros2_web_bridge.py`를 실행하여 웹과 ROS2 간 연동 가능

---

## 📡 주요 노드 및 기능

| 파일                    | 설명                             |
|-------------------------|----------------------------------|
| `puma_controller.py`     | 로봇 관절 제어 노드               |
| `joint_state_logger.py`  | 관절 상태 로그 기록              |
| `ros2_web_bridge.py`     | ROS2 ↔ 웹 브릿지 (토픽 통신)     |
| `puma_web_ui.html`       | 웹에서 관절 제어를 위한 인터페이스 |
| `display.launch.py`      | RViz에서 로봇 시각화를 위한 런치  |

---

## 🔧 향후 개선 방향

- FastAPI 서버를 통한 실시간 웹 제어 기능 통합
- Three.js 또는 Webviz를 통한 3D 시각화
- 로봇의 움직임 이력 로그를 DB에 저장

---

## 🧑‍💻 제작자

- 박준렬
- email : qkrwnsfuf123@gmail.com

---


# PUMA 3-DOF Robot System (ROS2 + Web UI)

This project is a 3-degree-of-freedom PUMA-type robot system built on ROS 2 (Humble). It includes a URDF model, RViz visualization, and a web-based control interface for real-time joint manipulation and logging.

---

## 📂 Project Structure

```bash
puma_robot/
├── CMakeLists.txt
├── package.xml
├── setup.py
├── urdf/
│   └── puma_3dof.urdf.xacro        # Robot URDF definition (xacro)
├── launch/
│   └── display.launch.py           # Launch file for URDF and RViz
├── scripts/
│   ├── puma_controller.py          # ROS2 control node
│   ├── joint_state_logger.py       # Joint state logging script
│   ├── ros2_web_bridge.py          # Web-ROS2 communication bridge
│   ├── test.py                     # Test script
│   ├── test.html                   # Test webpage
│   ├── puma_web_ui.html            # Web UI for robot control
│   └── logger_web_ui.html          # Web UI for logging status
├── rviz/
│   └── puma_config.rviz            # RViz configuration file
```

---

## 🛠️ Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10+
- xacro, joint_state_publisher, robot_state_publisher
- Web browser (Chrome recommended)

---

## 🧩 How to Run

### 1. Build ROS 2 Package

```bash
cd ~/ros2_ws
colcon build --packages-select puma_robot
source install/setup.bash
```

### 2. Visualize Robot in RViz

```bash
ros2 launch puma_robot display.launch.py
```

### 3. Use Web UI (Open HTML Directly)

```bash
# Open puma_web_ui.html or logger_web_ui.html in browser
open scripts/puma_web_ui.html
```

> Or run `ros2_web_bridge.py` for ROS2-Web integration

---

## 📡 Main Nodes and Features

| File                   | Description                           |
|------------------------|---------------------------------------|
| `puma_controller.py`    | Robot joint control node              |
| `joint_state_logger.py` | Joint state logger                   |
| `ros2_web_bridge.py`    | ROS2 ↔ Web communication bridge       |
| `puma_web_ui.html`      | UI for joint control in browser       |
| `display.launch.py`     | Launch RViz with robot model          |

---

## 🔧 Future Improvements

- Integrate FastAPI for full web-based control
- Add 3D visualization using Three.js or Webviz
- Store movement history in a database

---

## 🧑‍💻 Author

- JunRyeol Park
- email : qkrwnsfuf123@gmail.com

---
