#!/usr/bin/env bash
set -e -o pipefail
source ~/ros2_ws/install/setup.bash

echo "[B] /box_event /box_count 모니터링 시작 (RELIABLE)."
echo "[B] /session_summary 수신 시 자동 종료."

# (1) ros2 daemon이 꼬였으면 타입 조회에서 xmlrpc Fault가 잘 남 → 리셋
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true

# (2) 토픽이 실제로 뜰 때까지 잠깐 대기 (타입 못찾아 터지는 것 방지)
timeout 5 bash -c 'until ros2 topic list | grep -q "^/box_event$"; do sleep 0.1; done' || true
timeout 5 bash -c 'until ros2 topic list | grep -q "^/session_summary$"; do sleep 0.1; done' || true

# (3) echo들을 백그라운드로 띄움 (RELIABLE 고정)
ros2 topic echo /box_event std_msgs/msg/String \
  --qos-reliability reliable --qos-durability volatile &
PID1=$!

ros2 topic echo /box_count std_msgs/msg/Int32 \
  --qos-reliability reliable --qos-durability volatile &
PID2=$!

# (4) session_summary 한 번 오면 종료 트리거
ros2 topic echo /session_summary std_msgs/msg/String \
  --qos-reliability reliable --qos-durability volatile \
  --once >/dev/null 2>&1 || true

echo "[B] session_summary 수신 -> 모니터 종료"

# (5) 강제 kill 말고 SIGINT로 예쁘게 종료 (스택트레이스 방지)
kill -INT "$PID1" 2>/dev/null || true
kill -INT "$PID2" 2>/dev/null || true
wait "$PID1" "$PID2" 2>/dev/null || true

