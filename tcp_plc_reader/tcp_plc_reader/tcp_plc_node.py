# -*- coding: utf-8 -*-
import socket
import struct
import time
import threading
from typing import Optional, List

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)
from rclpy.executors import SingleThreadedExecutor
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float32, String, Int32, Float32MultiArray


class SimpleBoxGapDetector:
    """
    직전값 비교 기반 전이 판정 + 'BOX OFF 이후 다음 ON 타임아웃' 로직 포함
      - 상태: False=갭, True=박스
      - 갭→박스: (last_v - v) >= THRESH_DOWN_MM
      - 박스→갭: (v - last_v) >= THRESH_UP_MM
      - 길이 = (ON~OFF 시간) * line_speed
      - 갭   = (OFF~다음 ON 시간) * line_speed
    + 세션 내 누적 합계(box_total_mm, gap_total_mm) 실시간 갱신
    """

    def __init__(
        self,
        line_speed_mm_s: float,
        th_up: float,
        th_down: float,
        next_on_timeout_s: float,
    ):
        self.line_speed = float(line_speed_mm_s)
        self.THRESH_UP_MM = float(th_up)
        self.THRESH_DOWN_MM = float(th_down)
        self.next_on_timeout_s = float(next_on_timeout_s)
        self.reset()

    def reset(self):
        self.on_state = False
        self.last_v: Optional[float] = None
        self.t_on: Optional[float] = None
        self.t_gap_start: Optional[float] = None
        self.box_count = 0
        self.curr_box_id = 0

        # 타임아웃 상태
        self.last_box_off_s: Optional[float] = None
        self.waiting_for_next_on: bool = False

        # 세션 내 누적 합계
        self.box_total_mm: float = 0.0
        self.gap_total_mm: float = 0.0

    def totals(self):
        box_sum = self.box_total_mm
        gap_sum = self.gap_total_mm
        total = box_sum + gap_sum
        return box_sum, gap_sum, total

    def step(self, v: float, now_s: float) -> Optional[str]:
        evs: List[str] = []

        if self.last_v is None:
            self.last_v = v
            return None

        if not self.on_state:
            # 갭 상태 → 충분히 하락하면 박스 시작
            if (self.last_v - v) >= self.THRESH_DOWN_MM:
                # 이전 갭 길이 계산
                if self.t_gap_start is not None:
                    gap_mm = self.line_speed * (now_s - self.t_gap_start)
                    gap_mm = max(0.0, gap_mm)
                    self.gap_total_mm += gap_mm
                    evs.append(
                        f"[GAP] -> next id={self.curr_box_id+1} gap={gap_mm:.1f} mm"
                    )
                    self.t_gap_start = None

                self.on_state = True
                self.curr_box_id += 1
                self.t_on = now_s
                evs.append(f"[BOX ON] id={self.curr_box_id}")

                # 다음 ON을 기다리던 상태 해제
                self.waiting_for_next_on = False
                self.last_box_off_s = None

        else:
            # 박스 상태 → 충분히 상승하면 박스 종료
            if (v - self.last_v) >= self.THRESH_UP_MM:
                if self.t_on is not None:
                    L = self.line_speed * (now_s - self.t_on)
                    L = max(0.0, L)
                    self.box_total_mm += L
                    evs.append(f"[LEN] id={self.curr_box_id} L={L:.1f} mm")

                self.box_count += 1
                evs.append(f"[BOX OFF] id={self.curr_box_id} count={self.box_count}")

                self.on_state = False
                self.t_gap_start = now_s
                self.t_on = None

                # 타임아웃 타이머 시작
                self.last_box_off_s = now_s
                self.waiting_for_next_on = True

        self.last_v = v

        if evs:
            b, g, t = self.totals()
            evs.append(f"[TOTAL] box_sum={b:.1f} mm | gap_sum={g:.1f} mm | sum={t:.1f} mm")
        return " | ".join(evs) if evs else None

    def check_timeout(self, now_s: float) -> Optional[str]:
        """
        BOX OFF 이후 next_on_timeout_s 초 내 다음 ON이 없으면
        알림 이벤트 문자열 반환 (세션은 노드 쪽에서 종료)
        """
        if self.waiting_for_next_on and self.last_box_off_s is not None:
            elapsed = now_s - self.last_box_off_s
            if elapsed >= self.next_on_timeout_s:
                self.waiting_for_next_on = False
                b, g, t = self.totals()
                return (
                    f"[AUTO-EXIT] 마지막 BOX OFF 이후 {elapsed:.2f}s 경과 "
                    f"(임계 {self.next_on_timeout_s:.2f}s). "
                    f"[TOTAL] box_sum={b:.1f} mm | gap_sum={g:.1f} mm | sum={t:.1f} mm"
                )
        return None


class TcpPlcNode(Node):
    """
    PLC(TCP/IP) 수신 → L1(mm) 퍼블리시 → 세션 동안 박스/갭/누적합 계산

    ✅ 이번 수정 핵심:
    - auto_start_on_box=True 이면
      세션 OFF 상태에서도 L1을 감시하다가,
      "갭→박스" 전이 조건(THRESH_DOWN)을 만족하는 순간 자동으로 세션을 시작한다.
    - 박스가 끝나고 5초(next_on_timeout_s) 동안 다음 박스가 없으면
      세션을 종료하고(요약 publish) 다시 대기 상태로 돌아간다.
    - 즉, "노드는 계속 살아있고", "세션만 자동으로 반복" 된다.
    """

    def __init__(self):
        super().__init__("tcp_plc_node")

        # -------- Parameters --------
        # 통신/스케일/채널
        self.declare_parameter("plc_host", "10.10.10.1")
        self.declare_parameter("plc_port", 9000)
        self.declare_parameter("int_scale", 100.0)
        self.declare_parameter("idx_l1", 10)

        # 검출 파라미터
        self.declare_parameter("line_speed_mm_s", 82.7)
        self.declare_parameter("thresh_up_mm", 60.0)
        self.declare_parameter("thresh_down_mm", 60.0)

        # 타임아웃 파라미터
        self.declare_parameter("next_on_timeout_s", 5.0)

        # ✅ 자동 시작(세션 OFF에서 박스 들어오면 자동 START)
        self.declare_parameter("auto_start_on_box", True)
        # ✅ 노이즈/연속 트리거 방지용(자동 START 최소 간격)
        self.declare_parameter("auto_start_debounce_s", 0.5)

        # (권장 X) 타임아웃 시 노드까지 죽이기
        self.declare_parameter("auto_shutdown_on_timeout", False)

        # 소켓/재접속
        self.declare_parameter("recv_timeout_s", 0.1)
        self.declare_parameter("connect_timeout_s", 2.0)
        self.declare_parameter("reconnect_delay_s", 2.0)

        # 시뮬레이터
        self.declare_parameter("simulate", False)

        # 디버그
        self.declare_parameter("debug_dump", False)
        self.declare_parameter("bypass_frame_check", False)
        self.declare_parameter("scan_head_n", 16)

        # -------- Parameter 읽기 --------
        self.host = self.get_parameter("plc_host").get_parameter_value().string_value
        self.port = int(self.get_parameter("plc_port").get_parameter_value().integer_value)
        self.int_scale = float(self.get_parameter("int_scale").get_parameter_value().double_value)
        self.idx_l1 = int(self.get_parameter("idx_l1").get_parameter_value().integer_value)

        line_speed = float(self.get_parameter("line_speed_mm_s").get_parameter_value().double_value)
        th_up = float(self.get_parameter("thresh_up_mm").get_parameter_value().double_value)
        th_dn = float(self.get_parameter("thresh_down_mm").get_parameter_value().double_value)
        next_to = float(self.get_parameter("next_on_timeout_s").get_parameter_value().double_value)

        self.auto_start_on_box = bool(self.get_parameter("auto_start_on_box").get_parameter_value().bool_value)
        self.auto_start_debounce_s = float(
            self.get_parameter("auto_start_debounce_s").get_parameter_value().double_value
        )

        self.auto_shutdown_on_timeout = bool(
            self.get_parameter("auto_shutdown_on_timeout").get_parameter_value().bool_value
        )

        self.recv_timeout = float(self.get_parameter("recv_timeout_s").get_parameter_value().double_value)
        self.connect_timeout = float(self.get_parameter("connect_timeout_s").get_parameter_value().double_value)
        self.reconnect_delay = float(self.get_parameter("reconnect_delay_s").get_parameter_value().double_value)

        self.simulate = bool(self.get_parameter("simulate").get_parameter_value().bool_value)
        self.debug_dump = bool(self.get_parameter("debug_dump").get_parameter_value().bool_value)
        self.bypass_frame_check = bool(self.get_parameter("bypass_frame_check").get_parameter_value().bool_value)
        self.scan_head_n = int(self.get_parameter("scan_head_n").get_parameter_value().integer_value)

        # 검출기 (세션 단위로 reset)
        self.detector = SimpleBoxGapDetector(line_speed, th_up, th_dn, next_to)

        # -------- QoS 설정 --------
        qos_best = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # -------- Publishers --------
        self.pub_l1 = self.create_publisher(Float32, "laser/l1", qos_best)

        self.pub_event = self.create_publisher(String, "box_event", qos_reliable)
        self.pub_count = self.create_publisher(Int32, "box_count", qos_reliable)

        self.pub_rx_hex = self.create_publisher(String, "plc/rx_hex", qos_best)
        self.pub_raw_vals = self.create_publisher(Float32MultiArray, "laser/raw_values", qos_best)

        self.pub_box_total = self.create_publisher(Float32, "box_total_mm", qos_reliable)
        self.pub_gap_total = self.create_publisher(Float32, "gap_total_mm", qos_reliable)
        self.pub_total = self.create_publisher(Float32, "total_mm", qos_reliable)

        # ✅ 요약도 RELIABLE로 맞춰두는 게 안전
        self.pub_summary = self.create_publisher(String, "session_summary", qos_reliable)

        # -------- 세션 제어 --------
        self.session_active: bool = False
        self._session_start_wall: Optional[float] = None
        self._final_reason: str = "shutdown"

        self.sub_cmd = self.create_subscription(String, "measure_cmd", self._on_measure_cmd, 10)

        # -------- Runtime state --------
        self._sock: Optional[socket.socket] = None
        self._running: bool = True

        # perf_counter 기반 시간축(샘플/검출용)
        self._t0 = time.perf_counter()

        # ✅ 세션 OFF 상태에서 자동 START 판단용 last_v
        self._idle_last_v: Optional[float] = None
        self._last_auto_start_s: float = -1e9

        # 타임아웃 감시 타이머 (50ms 주기)
        self._timeout_timer = self.create_timer(0.05, self._check_timeout)

        # 파라미터 런타임 반영
        self.add_on_set_parameters_callback(self._on_param_set)

        # RX thread (daemon=False로 두고 종료 시 join)
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=False)

        self.get_logger().info(
            f"TCP PLC Node starting (host={self.host}, port={self.port}, idx_l1={self.idx_l1}, simulate={self.simulate})"
        )
        self.get_logger().info(
            "대기 상태입니다. /measure_cmd 'START' 또는 auto_start_on_box=True이면 박스 감지 시 자동 세션 시작."
        )
        self._rx_thread.start()

    # ---------- 세션 제어 ----------
    def _on_measure_cmd(self, msg: String):
        cmd = msg.data.strip().upper()
        if cmd == "START":
            if self.session_active:
                self.get_logger().warn("이미 세션이 진행 중입니다. START 무시.")
            else:
                # 수동 START는 “즉시 세션 시작(리셋)”만 하고,
                # 첫 박스는 들어오는 샘플로 자연스럽게 잡도록 둔다.
                self._start_session(reason="manual_start")
        elif cmd == "STOP":
            if self.session_active:
                self._end_session(reason="manual_stop")
            else:
                self.get_logger().warn("진행 중인 세션이 없어 STOP 무시.")
        else:
            self.get_logger().warn(f"알 수 없는 measure_cmd: {msg.data}")

    def _start_session(self, reason: str, seed_last_v: Optional[float] = None):
        # 세션 단위 리셋
        self.detector.reset()
        if seed_last_v is not None:
            # 자동 START에서 “첫 샘플에서 BOX ON 이벤트”를 만들기 위해 last_v를 심어둠
            self.detector.last_v = float(seed_last_v)

        self.session_active = True
        self._session_start_wall = time.perf_counter()
        self._final_reason = reason

        txt = "[SESSION] NEW SESSION START"
        self._safe_publish(self.pub_event, String(data=txt))
        self.get_logger().info(txt)

    def _end_session(self, reason: str):
        if not self.session_active:
            return

        self.session_active = False
        self._final_reason = reason

        # 세션 요약 출력/퍼블리시
        self._publish_summary()

        # ✅ 다음 자동 시작을 위해 idle 감시 상태로 복귀
        self._idle_last_v = None

    # ---------- Parameter callback ----------
    def _on_param_set(self, params):
        try:
            for p in params:
                if p.name == "line_speed_mm_s":
                    val = float(p.value)
                    if val <= 0:
                        return SetParametersResult(successful=False, reason="line_speed must be > 0")
                    self.detector.line_speed = val
                elif p.name == "thresh_up_mm":
                    self.detector.THRESH_UP_MM = float(p.value)
                elif p.name == "thresh_down_mm":
                    self.detector.THRESH_DOWN_MM = float(p.value)
                elif p.name == "idx_l1":
                    self.idx_l1 = int(p.value)
                elif p.name == "int_scale":
                    self.int_scale = float(p.value)
                elif p.name == "next_on_timeout_s":
                    self.detector.next_on_timeout_s = float(p.value)
                elif p.name == "auto_shutdown_on_timeout":
                    self.auto_shutdown_on_timeout = bool(p.value)
                elif p.name == "auto_start_on_box":
                    self.auto_start_on_box = bool(p.value)
                elif p.name == "auto_start_debounce_s":
                    self.auto_start_debounce_s = float(p.value)
                elif p.name == "debug_dump":
                    self.debug_dump = bool(p.value)
                elif p.name == "bypass_frame_check":
                    self.bypass_frame_check = bool(p.value)
                elif p.name == "scan_head_n":
                    self.scan_head_n = int(p.value)
            return SetParametersResult(successful=True)
        except Exception as e:
            return SetParametersResult(successful=False, reason=str(e))

    # ---------- Timeout watcher ----------
    def _check_timeout(self):
        if not self.session_active:
            return
        now_s = time.perf_counter() - self._t0
        msg_txt = self.detector.check_timeout(now_s)
        if msg_txt:
            self._safe_publish(self.pub_event, String(data=msg_txt))
            self.get_logger().warn(msg_txt)

            # 세션 종료
            self._end_session(reason="timeout")

            # ⚠️ 권장 X: 노드까지 죽이는 옵션(원래 너가 겪던 Bad file descriptor/RCLError 원인)
            if self.auto_shutdown_on_timeout:
                self.get_logger().warn("auto_shutdown_on_timeout=True → 노드까지 종료(권장 X)")
                self._running = False
                try:
                    self._timeout_timer.cancel()
                except Exception:
                    pass

    # ---------- publish helper (shutdown 중 publish 방지) ----------
    def _safe_publish(self, pub, msg):
        try:
            if not self._running:
                return
            if not rclpy.ok():
                return
            # context가 죽은 뒤 publish하면 "publisher's context is invalid" 터짐
            if not self.context.ok():
                return
            pub.publish(msg)
        except Exception:
            # shutdown 직전 경쟁상태는 조용히 무시
            return

    # ---------- 내부 유틸 ----------
    def _publish_totals(self):
        b, g, t = self.detector.totals()
        self._safe_publish(self.pub_box_total, Float32(data=float(b)))
        self._safe_publish(self.pub_gap_total, Float32(data=float(g)))
        self._safe_publish(self.pub_total, Float32(data=float(t)))

    def _publish_summary(self):
        b, g, t = self.detector.totals()
        if self._session_start_wall is not None:
            elapsed = time.perf_counter() - self._session_start_wall
        else:
            elapsed = 0.0

        msg = String()
        msg.data = (
            f"[SUMMARY] reason={self._final_reason} | "
            f"boxes={self.detector.box_count} | "
            f"box_sum={b:.1f} mm | gap_sum={g:.1f} mm | "
            f"total={t:.1f} mm | elapsed={elapsed:.2f}s"
        )
        self._safe_publish(self.pub_summary, msg)
        self.get_logger().info(msg.data)

    # ---------- TCP helpers ----------
    def _close_socket(self):
        if not self._sock:
            return
        try:
            self._sock.close()
        except Exception:
            pass
        finally:
            self._sock = None

    def _connect(self) -> Optional[socket.socket]:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(self.connect_timeout)
        try:
            s.connect((self.host, self.port))
            s.settimeout(self.recv_timeout)
            self.get_logger().info("TCP connected")
            return s
        except Exception as e:
            self.get_logger().warn(f"TCP connect failed: {e}")
            try:
                s.close()
            except Exception:
                pass
            return None

    @staticmethod
    def _parse_payload_to_values(payload: bytes, scale: float) -> List[float]:
        out: List[float] = []
        for i in range(0, len(payload), 4):
            chunk = payload[i : i + 4]
            if len(chunk) < 4:
                break
            out.append(struct.unpack("<i", chunk)[0] / scale)
        return out

    # ---------- RX Loop ----------
    def _rx_loop(self):
        rng = None
        if self.simulate:
            import random
            rng = random.Random(0)
            val = 500.0

        while self._running:
            if self.simulate:
                now_s = time.perf_counter() - self._t0
                step = rng.choice([-80, -2, 0, +2, +80])
                val = max(0.0, min(2000.0, val + step))
                self._handle_sample(val, now_s)
                time.sleep(self.recv_timeout)
                continue

            if not self._sock:
                sock = self._connect()
                if not sock:
                    time.sleep(self.reconnect_delay)
                    continue
                self._sock = sock

            try:
                data = self._sock.recv(1024)
                if not data:
                    self.get_logger().warn("TCP peer closed")
                    self._close_socket()
                    time.sleep(self.reconnect_delay)
                    continue

                if self.debug_dump:
                    hx = data[:32].hex(" ")
                    self._safe_publish(self.pub_rx_hex, String(data=f"len={len(data)} head={hx}"))

                if not self.bypass_frame_check:
                    if len(data) < 3 or data[0] != 0x02 or data[-1] != 0x03:
                        self.get_logger().debug("Bad frame (skip)")
                        continue
                    payload = data[2:-1]
                else:
                    payload = data

                values = self._parse_payload_to_values(payload, self.int_scale)

                if self.debug_dump and values:
                    head = values[: max(0, min(self.scan_head_n, len(values)))]
                    arr = Float32MultiArray()
                    arr.data = [float(v) for v in head]
                    self._safe_publish(self.pub_raw_vals, arr)

                if len(values) > self.idx_l1:
                    now_s = time.perf_counter() - self._t0
                    l1 = values[self.idx_l1]
                    self._handle_sample(l1, now_s)

            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f"RX error: {e}")
                self._close_socket()
                time.sleep(self.reconnect_delay)
                continue

        # 루프 종료 시 소켓 정리
        self._close_socket()

    # ---------- 샘플 처리 ----------
    def _handle_sample(self, l1_mm: float, now_s: float):
        # 1) L1 퍼블리시 (세션 여부와 상관 없이)
        self._safe_publish(self.pub_l1, Float32(data=float(l1_mm)))

        # 2) 세션 OFF면: 자동 START 여부 판단
        if not self.session_active:
            if not self.auto_start_on_box:
                # 수동 START만 쓰는 모드면 여기서 끝
                self._idle_last_v = float(l1_mm)
                return

            prev = self._idle_last_v
            self._idle_last_v = float(l1_mm)

            if prev is None:
                return

            # 자동 START 디바운스
            if (now_s - self._last_auto_start_s) < self.auto_start_debounce_s:
                return

            # 갭→박스 전이 조건(하락)
            if (prev - float(l1_mm)) >= self.detector.THRESH_DOWN_MM:
                # ✅ 자동 세션 시작: prev를 seed로 넣어서 첫 step에서 BOX ON 이벤트가 뜨게 함
                self._last_auto_start_s = now_s
                self._start_session(reason="auto_start", seed_last_v=prev)

                # 세션을 막 켰으니, 바로 이 샘플을 detector에 넣어 이벤트를 만들기
                ev = self.detector.step(float(l1_mm), now_s)
                if ev:
                    self._safe_publish(self.pub_event, String(data=ev))
                    self._publish_totals()
                return

            return

        # 3) 세션 ON 상태에서만 박스/갭/누적 계산
        ev = self.detector.step(float(l1_mm), now_s)
        if ev:
            self._safe_publish(self.pub_event, String(data=ev))

            # 누적 합계 퍼블리시
            self._publish_totals()

            # 박스 카운트 업데이트
            if "BOX OFF" in ev:
                self._safe_publish(self.pub_count, Int32(data=int(self.detector.box_count)))

    # ---------- Lifecycle ----------
    def destroy_node(self):
        # 종료 순서: running false → 타이머 cancel → 소켓 close → thread join → 세션 정리
        self._running = False
        try:
            self._timeout_timer.cancel()
        except Exception:
            pass

        self._close_socket()

        # 세션이 열려 있었다면 요약 한 번 더
        if self.session_active:
            self._end_session(reason="node_shutdown")

        # RX thread 정리
        try:
            if self._rx_thread.is_alive():
                self._rx_thread.join(timeout=2.0)
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TcpPlcNode()

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        while rclpy.ok() and node._running:
            executor.spin_once(timeout_sec=0.1)
    finally:
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()

