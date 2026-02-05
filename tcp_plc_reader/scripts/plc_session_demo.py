# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String, Int32, Float32


class PlcSessionDemo(Node):
    """
    - /measure_cmd 로 'START' 한번 쏴서 세션 시작
    - /box_event, /box_count, /total_mm 을 구독해서 결과 확인
    """

    def __init__(self):
        super().__init__('plc_session_demo')

        # 결과 토픽과 QoS 맞추기 위해 RELIABLE
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 세션 제어 퍼블리셔
        self.measure_cmd_pub = self.create_publisher(
            String,
            'measure_cmd',
            10,  # 기본 QoS (RELIABLE)
        )

        # 결과 구독
        self.sub_event = self.create_subscription(
            String, 'box_event', self.on_event, qos_reliable
        )
        self.sub_count = self.create_subscription(
            Int32, 'box_count', self.on_count, qos_reliable
        )
        self.sub_total = self.create_subscription(
            Float32, 'total_mm', self.on_total, qos_reliable
        )

        # START를 한 번만 보내기 위한 타이머
        self.start_sent = False
        self.timer_start = self.create_timer(1.0, self.send_start_once)

        # (옵션) 일정 시간 후 STOP 보내고 싶으면 사용
        # self.timer_stop = self.create_timer(10.0, self.send_stop_once)
        # self.stop_sent = False

        self.get_logger().info("PlcSessionDemo 시작. 1초 후 /measure_cmd='START' 전송 예정.")

    def send_start_once(self):
        if self.start_sent:
            return
        msg = String()
        msg.data = "START"
        self.measure_cmd_pub.publish(msg)
        self.start_sent = True
        self.get_logger().info(">>> /measure_cmd = 'START' 전송 완료.")

    def send_stop_once(self):
        # STOP 자동 전송 쓰고 싶으면, 위에서 타이머/플래그 활성화
        if getattr(self, "stop_sent", False):
            return
        msg = String()
        msg.data = "STOP"
        self.measure_cmd_pub.publish(msg)
        self.stop_sent = True
        self.get_logger().info(">>> /measure_cmd = 'STOP' 전송 완료.")

    def on_event(self, msg: String):
        self.get_logger().info(f"[box_event] {msg.data}")

    def on_count(self, msg: Int32):
        self.get_logger().info(f"[box_count] {msg.data}")

    def on_total(self, msg: Float32):
        self.get_logger().info(f"[total_mm] {msg.data:.1f} mm")


def main(args=None):
    rclpy.init(args=args)
    node = PlcSessionDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

