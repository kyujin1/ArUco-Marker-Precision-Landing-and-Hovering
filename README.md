ArUco-Marker-Precision-Landing-and-Hovering
한국로봇항공기 경연대회 본선 진출 프로젝트

개요
드론의 하단 카메라를 이용해 ArUco 마커를 인식하고, 이를 기반으로 정밀 착륙 및 호버링을 수행하는 ROS2 패키지입니다.

⚙️ 핵심 기술 및 구현 내용
ROS2 Humble/Foxy 표준 빌드 시스템: colcon 및 파이썬 패키지 구조를 적용하여 유지보수성 향상

비전 데이터 처리: OpenCV를 활용한 마커 Pose(x, y, z) 실시간 추출 및 제어 노드 통신

PX4 Offboard Control: 비행 제어 노드와 비전 노드의 모듈화된 설계를 통한 정밀 제어 구현

시스템 자동화: ros2 launch를 통한 착륙/호버링 시나리오별 통합 실행 환경 구축

실행 방법
Bash
# 정밀 착륙 시나리오 실행
ros2 launch vtol precision_land.launch.py

# 정밀 호버링 시나리오 실행
ros2 launch vtol precision_hover.launch.py
