#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32


class PlcIntegratedMain(Node):
    """
    통합코드 노드 (자동 반복 세션 버전)

    기능:
      - 주기적으로 세션 상태를 확인하고, 세션이 안 돌고 있으면
        /measure_cmd 에 'START' 를 자동 전송해 다음 세션 시작
      - /box_event, /box_count, /box_total_mm, /gap_total_mm, /total_mm,
        /session_summary 를 구독해서 내용 로그 출력
    """

    def __init__(self):
        super().__init__('plc_integrated_main')

        # --- START/STOP 명령 퍼블리셔 (/measure_cmd) ---
        self.cmd_pub = self.create_publisher(String, 'measure_cmd', 10)

        # --- 결과/이벤트 구독 ---
        self.event_sub = self.create_subscription(
            String, 'box_event', self.on_box_event, 10
        )
        self.count_sub = self.create_subscription(
            Int32, 'box_count', self.on_box_count, 10
        )
        self.box_total_sub = self.create_subscription(
            Float32, 'box_total_mm', self.on_box_total, 10
        )
        self.gap_total_sub = self.create_subscription(
            Float32, 'gap_total_mm', self.on_gap_total, 10
        )
        self.total_sub = self.create_subscription(
            Float32, 'total_mm', self.on_total, 10
        )
        self.summary_sub = self.create_subscription(
            String, 'session_summary', self.on_summary, 10
        )

        # --- 세션 상태/자동 재시작 플래그 ---
        self.session_running = False      # 지금 세션이 돌고 있다고 보는지
        self.auto_restart = True          # 세션 끝나면 자동 재시작 여부

        # 1초마다 세션 상태 보고 필요하면 START 쏘는 타이머
        self.start_timer = self.create_timer(1.0, self._auto_start_loop)

        self.get_logger().info("PlcIntegratedMain 노드 시작 (자동 반복 세션 모드).")

    # ---------- 주기적으로 세션 상태 확인 후 START ----------
    def _auto_start_loop(self):
        if not self.auto_restart:
            return

        if self.session_running:
            # 이미 세션 돌고 있으면 아무것도 안 함
            return

        # 세션이 안 돌고 있으면 새로 START 전송
        msg = String()
        msg.data = 'START'
        self.cmd_pub.publish(msg)
        self.session_running = True
        self.get_logger().info("[통합코드] measure_cmd='START' 자동 전송 (새 세션 시작).")

    # ---------- 콜백들 ----------
    def on_box_event(self, msg: String):
        txt = msg.data
        self.get_logger().info(f"[box_event] {txt}")
        # TODO: 여기서 txt 파싱해서 상위 로직에 넘기기

    def on_box_count(self, msg: Int32):
        cnt = msg.data
        self.get_logger().info(f"[box_count] {cnt}")

    def on_box_total(self, msg: Float32):
        box_sum = msg.data
        self.get_logger().info(f"[box_total_mm] {box_sum:.1f} mm")

    def on_gap_total(self, msg: Float32):
        gap_sum = msg.data
        self.get_logger().info(f"[gap_total_mm] {gap_sum:.1f} mm")

    def on_total(self, msg: Float32):
        total = msg.data
        self.get_logger().info(f"[total_mm] {total:.1f} mm")

    def on_summary(self, msg: String):
        # 세션 하나 끝났다는 의미 → 다음 타이머 턴에서 새 START 나오게 플래그 내림
        self.session_running = False
        self.get_logger().info(f"[session_summary] {msg.data}")
        # 여기서 DB 저장/로그 파일 등 처리하면 됨.


def main(args=None):
    rclpy.init(args=args)
    node = PlcIntegratedMain()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
