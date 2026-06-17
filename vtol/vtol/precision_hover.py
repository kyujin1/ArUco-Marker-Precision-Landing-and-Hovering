import rclpy
from rclpy.node import Node
import time
import math

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition
)

from std_msgs.msg import Float32MultiArray


class MarkerFollower(Node):

    def __init__(self):
        super().__init__('marker_follower')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # =========================
        # SUB
        # =========================
        self.create_subscription(
            Float32MultiArray,
            '/marker/pose',
            self.marker_callback,
            10
        )

        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.local_position_callback,
            qos
        )

        # =========================
        # PUB
        # =========================
        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10
        )

        self.traj_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10
        )

        self.cmd_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )

        # =========================
        # STATE
        # =========================
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = -10.0

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = None

        self.alt_locked = False
        self.pos_ready = False

        self.counter = 0
        self.armed = False
        self.offboard_set = False
        
        # Loiter 관련 상태 변수
        self.loiter_started = False 

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("🚀 Marker Follower & Infinite Loiter Started")

    # =========================
    # POSITION
    # =========================
    def local_position_callback(self, msg):
        self.pos_ready = True

        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z

        if not self.alt_locked:
            self.target_z = msg.z
            self.alt_locked = True
            self.get_logger().info(f"🔒 ALTITUDE LOCK: {self.target_z:.2f}")

    # =========================
    # MARKER
    # =========================
    def marker_callback(self, msg):
        if len(msg.data) < 4:
            return

        if msg.data[3] != 1.0:
            return

        self.target_x = float(msg.data[0])
        self.target_y = float(msg.data[1])

    # =========================
    # COMMAND
    # =========================
    def cmd(self, c, p1=0.0, p2=0.0, p3=0.0):
        msg = VehicleCommand()
        msg.command = c
        msg.param1 = float(p1)
        msg.param2 = float(p2)
        msg.param3 = float(p3)
        msg.target_system = 1
        msg.target_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(msg)

    def arm(self):
        self.get_logger().info("🟢 ARM")
        self.cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def offboard(self):
        self.get_logger().info("🟡 OFFBOARD")
        self.cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    def loiter(self):
        self.get_logger().info("🎯 TARGET REACHED! Switching to LOITER Mode (Maintaining...)")
        # Main Mode: 4(AUTO), Sub Mode: 3(LOITER)
        self.cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 3.0)
        self.loiter_started = True

    # =========================
    # LOOP
    # =========================
    def timer_callback(self):

        if not self.pos_ready or self.target_z is None:
            return

        # Loiter 모드 진입 후 로직
        if self.loiter_started:
            # 20초 타이머 로직을 제거하고, 상태 메시지만 간헐적으로 출력합니다.
            self.counter += 1
            if self.counter % 40 == 0: # 약 2초마다 상태 보고
                self.get_logger().info("🛑 Loiter Mode Active: Holding Position...")
            return

        # =========================
        # OFFBOARD HEARTBEAT
        # =========================
        offboard = OffboardControlMode()
        offboard.position = True
        offboard.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(offboard)

        # =========================
        # CONTROL
        # =========================
        gain = 0.2

        dx = -gain * self.target_y
        dy = gain * self.target_x

        set_x = self.current_x + dx
        set_y = self.current_y + dy

        # =========================
        # SETPOINT
        # =========================
        traj = TrajectorySetpoint()
        traj.position = [
            float(set_x),
            float(set_y),
            float(self.target_z)
        ]
        traj.yaw = 0.0
        traj.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        self.traj_pub.publish(traj)

        # =========================
        # OFFBOARD FLOW
        # =========================
        self.counter += 1

        if self.counter == 20:
            self.offboard_set = True
            self.offboard()

        if self.counter == 40:
            self.armed = True
            self.arm()

        # =========================
        # ERROR LOGGING & CONDITION CHECK
        # =========================
        error_dist = math.sqrt(self.target_x**2 + self.target_y**2)

        if self.counter % 10 == 0:
            self.get_logger().info(f"📍 Error XY: {error_dist:.3f}m (x: {self.target_x:.3f}, y: {self.target_y:.3f})")

        # 조건 확인: 오차 10cm 미만 시 Loiter 모드로 고정
        if self.armed and self.offboard_set and not self.loiter_started:
            if abs(self.target_x) < 0.1 and abs(self.target_y) < 0.1:
                self.loiter()


def main(args=None):
    rclpy.init(args=args)
    node = MarkerFollower()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("🛑 EMERGENCY LAND")
        node.cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        time.sleep(2)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
