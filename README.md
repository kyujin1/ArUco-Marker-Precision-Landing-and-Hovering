# ArUco Marker Precision Landing and Hovering for PX4 VTOL

본 프로젝트는 PX4 펌웨어가 탑재된 VTOL(수직 이착륙기) 기체를 위한 ROS2 기반 정밀 착륙 및 호버링 시스템입니다. ArUco 마커를 인식하여 기체의 상대적 위치를 계산하고, Offboard 모드를 통해 정밀 제어를 수행합니다.

## 🛠 필수 요구사항 (Prerequisites)

본 프로젝트를 실행하기 위해서는 PX4와 ROS2 간의 통신을 위한 `px4_ros_com` 패키지가 반드시 필요합니다.

### 1. 환경 설정 및 의존성 설치

작업 디렉토리(`~/vtol_ws/src`)로 이동하여 PX4-ROS2 연동을 위한 공식 패키지를 클론합니다.

    cd ~/vtol_ws/src
    git clone https://github.com/PX4/px4_msgs.git
    git clone https://github.com/PX4/px4_ros_com.git

### 2. 빌드

워크스페이스 루트로 이동하여 빌드를 수행합니다.

    cd ~/vtol_ws
    colcon build --packages-select px4_msgs px4_ros_com vtol
    source install/setup.bash

## ⚙️ 시스템 아키텍처



* **cam_node**: 카메라 이미지에서 ArUco 마커의 Pose 데이터를 추출하여 발행
* **precision_land/hover**: PX4 Offboard 모드 제어 및 타겟 위치 기반 착륙/호버링 수행

## 🚀 실행 방법

### 정밀 착륙 시나리오

    cd ~/vtol_ws
    ros2 launch vtol precision_land.launch.py

### 정밀 호버링 시나리오

    cd ~/vtol_ws
    ros2 launch vtol precision_hover.launch.py

## ⚠️ 주의사항

* **PX4 설정**: 드론은 반드시 `Offboard` 모드 진입이 가능한 상태여야 하며, `vtol_parameter` 설정을 통해 수직 이착륙 모드가 활성화되어 있어야 합니다.
* **통신**: ROS2와 PX4 간의 `MicroXRCE-Agent`가 실행 중인지 확인하세요. (PX4 연동 공식 가이드: [PX4 ROS2 User Guide](https://docs.px4.io/main/ko/ros2/user_guide/))
