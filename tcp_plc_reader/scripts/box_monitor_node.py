# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32


class BoxMonitorNode(Node):
    """
    통합코드 예시 노드:
      - 노드 시작 시 자동으로 /measure_cmd = 'START' 1회 전송
      - /box_event, /box_count, /box_total_mm, /gap_total_mm, /total_mm 구독
      - 콜백에서 받은 값들을 통합 로직으로 넘기는 자리
    """

    def __init__(self):
        super().__init__('box_monitor_node')

        # --- Publisher: /measure_cmd (START/STOP) ---
        self.pub_cmd = self.create_publisher(String, 'measure_cmd', 10)

        # --- Subscribers: 결과 토픽들 ---
        self.sub_event = self.create_subscription(
            String, 'box_event', self.on_box_event, 10
        )
        self.sub_count = self.create_subscription(
            Int32, 'box_count', self.on_box_count, 10
        )
        self.sub_box_total = self.create_subscription(
            Float32, 'box_total_mm', self.on_box_total, 10
        )
        self.sub_gap_total = self.create_subscription(
            Float32, 'gap_total_mm', self.on_gap_total, 10
        )
        self.sub_total = self.create_subscription(
            Float32, 'total_mm', self.on_total, 10
        )

        # 노드 시작 후 조금 있다가 START 보내기 위한 타이머 (1초 딜레이)
        self.start_sent = False
        self.start_timer = self.create_timer(1.0, self._send_start_once)

        self.get_logger().info("BoxMonitorNode 시작")

    # ---------- START 한 번만 보내기 ----------
    def _send_start_once(self):
        if self.start_sent:
            return
        msg = String()
        msg.data = 'START'
        self.pub_cmd.publish(msg)
        self.start_sent = True
        self.get_logger().info("[BoxMonitorNode] measure_cmd='START' 전송 완료")
        # 필요 없으니 타이머 정지
        self.start_timer.cancel()

    # ---------- 콜백들 ----------
    def on_box_event(self, msg: String):
        txt = msg.data
        self.get_logger().info(f"[box_event] {txt}")

        # 여기서 txt 파싱해서 통합 로직으로 넘기면 됨
        # 예: [LEN], [GAP], [TOTAL]에 따라 분기 처리

    def on_box_count(self, msg: Int32):
        count = msg.data
        self.get_logger().info(f"[box_count] {count}")
        # 통합코드: 현재까지 처리한 박스 개수로 상태 갱신

    def on_box_total(self, msg: Float32):
        box_sum = msg.data
        self.get_logger().debug(f"[box_total_mm] {box_sum:.1f} mm")
        # 통합코드: 전체 박스 길이 누적 사용

    def on_gap_total(self, msg: Float32):
        gap_sum = msg.data
        self.get_logger().debug(f"[gap_total_mm] {gap_sum:.1f} mm")

    def on_total(self, msg: Float32):
        total = msg.data
        self.get_logger().info(f"[total_mm] {total:.1f} mm")
        # 예: total 이 어떤 기준 넘으면 다음 단계로 넘어가는 로직 등

def main(args=None):
    rclpy.init(args=args)
    node = BoxMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
