import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge

import cv2
import cv2.aruco as aruco
import numpy as np


class ArucoPoseNode(Node):
    def __init__(self):
        super().__init__('aruco_pose_node')

        # 카메라 구독
        self.subscription = self.create_subscription(
            Image,
            '/vtol/downward_camera/image_raw',
            self.listener_callback,
            10
        )

        # Pose 발행 (x, y, z, detected_flag)
        self.pose_pub = self.create_publisher(
            Float32MultiArray,
            '/marker/pose',
            10
        )

        self.bridge = CvBridge()

        self.get_logger().info('=== Aruco Pose Node Started ===')

    def listener_callback(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            h, w = frame.shape[:2]

            # 🔥 카메라 파라미터 (임시값)
            fx = w
            fy = h
            cx = w / 2
            cy = h / 2

            camera_matrix = np.array([
                [fx, 0, cx],
                [0, fy, cy],
                [0, 0, 1]
            ], dtype=np.float32)

            dist_coeffs = np.zeros((5, 1))  # 왜곡 없음 가정

            # Aruco 설정
            aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

            try:
                detector = aruco.ArucoDetector(aruco_dict, aruco.DetectorParameters())
                corners, ids, _ = detector.detectMarkers(frame)
            except:
                params = aruco.DetectorParameters_create()
                corners, ids, _ = aruco.detectMarkers(frame, aruco_dict, parameters=params)

            pose_msg = Float32MultiArray()

            if ids is not None:
                # 🔥 마커 실제 크기 (1.5m)
                marker_length = 1.5

                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners,
                    marker_length,
                    camera_matrix,
                    dist_coeffs
                )

                # 첫 번째 마커 기준
                tvec = tvecs[0][0]

                x = float(tvec[0])
                y = float(tvec[1])
                z = float(tvec[2])

                pose_msg.data = [x, y, z, 1.0]

                self.get_logger().info(
                    f'Pose → x:{x:.2f} m, y:{y:.2f} m, z:{z:.2f} m'
                )

                # 시각화
                aruco.drawDetectedMarkers(frame, corners, ids)
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[0], tvecs[0], 0.5)

            else:
                pose_msg.data = [0.0, 0.0, 0.0, 0.0]

            self.pose_pub.publish(pose_msg)

            cv2.imshow("Aruco Pose View", frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ArucoPoseNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
