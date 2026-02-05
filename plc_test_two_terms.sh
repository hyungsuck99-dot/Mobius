#!/usr/bin/env bash
set -euo pipefail

echo "[TEST] PLC 측정 + 모니터 터미널 2개 실행"

# 측정용 터미널(A)
gnome-terminal --title="PLC MEASURE (A)" -- bash -c "
  bash ~/plc_up.sh
  echo '[A] 측정 노드 종료. 창 닫으려면 Enter.'
  read
"

sleep 1

# 모니터용 터미널(B)
gnome-terminal --title="WATCH (B)" -- bash -c "
  bash ~/plc_watch.sh
  echo '[B] 모니터 종료. 창 닫으려면 Enter.'
  read
"

# ✅ START 자동 전송(노드가 완전히 뜰 시간을 조금 줌)
sleep 2
source ~/ros2_ws/install/setup.bash
ros2 topic pub -1 /measure_cmd std_msgs/msg/String "{data: 'START'}"
echo "[TEST] START 전송 완료"

