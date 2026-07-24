#!/usr/bin/env python3
"""
bartender_admin_control_bridge.py
──────────────────────────────────────────────────────────────
CoboTender 관리자 UI ↔ 바텐더 제어코드 ↔ Doosan 상태 모니터링 중계 노드

목적
- 관리자 UI에서 일시정지/재개/취소/비상정지/비상정지 해제/Recovery 명령을 하나의 토픽으로 받음
- 기존 제어코드가 이해할 수 있는 토픽으로 명령을 분배
- 제어코드가 publish하는 /dsr01/robot_monitor_status(JSON)를 받아 안전상태/로그를 정리
- Flask app.py는 /robot/admin_bridge_status를 구독하거나 기존 /dsr01/robot_monitor_status를 직접 구독해 UI에 표시 가능

필수 전제
- app.py가 /ui/admin_control(std_msgs/String)에 명령을 publish하도록 수정 필요
- 바텐더 제어코드가 /dsr01/task_control(std_msgs/String)을 구독해 PAUSE/RESUME/CANCEL을 처리하도록 최소 수정 필요
- 바텐더 제어코드가 /dsr01/robot_monitor_status(std_msgs/String JSON)를 publish해야 protective stop/e-stop 상태를 UI에 표시 가능
- 바텐더 제어코드가 /dsr01/recovery_command(std_msgs/String) "RECOVER"를 구독해야 recovery 버튼 동작 가능
"""

from __future__ import annotations

import json
import time
from collections import deque
from dataclasses import dataclass, asdict
from typing import Deque, Dict, Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


ROBOT_ID = "dsr01"

# UI -> bridge
ADMIN_CONTROL_TOPIC = "/ui/admin_control"          # std_msgs/String: PAUSE, RESUME, ESTOP, ESTOP_RELEASE, HOME_RETURN, RECOVER

# bridge -> current bartender control code / robot side
TASK_CONTROL_TOPIC = f"/{ROBOT_ID}/task_control"  # std_msgs/String: PAUSE, RESUME, HOME_RETURN, ESTOP_RELEASE_HOME
ESTOP_TOPIC = "/ui/emergency_stop"                # std_msgs/Bool: True stop, False release only
RECOVERY_TOPIC = f"/{ROBOT_ID}/recovery_command"  # std_msgs/String: RECOVER

# controller -> bridge -> UI
MONITOR_STATUS_TOPIC = f"/{ROBOT_ID}/robot_monitor_status"  # std_msgs/String JSON, from bartender control code
BRIDGE_STATUS_TOPIC = "/robot/admin_bridge_status"          # std_msgs/String JSON, simplified status for app.py


HW_MAP = {
    1: {"state": "STANDBY", "label": "정상 대기", "level": "OK", "recovery_required": False},
    2: {"state": "MOVING", "label": "이동 중", "level": "INFO", "recovery_required": False},
    3: {"state": "SAFE_OFF", "label": "서보 꺼짐", "level": "ERROR", "recovery_required": True},
    5: {"state": "PROT_STOP", "label": "안전정지", "level": "WARN", "recovery_required": True},
    6: {"state": "EMRG_STOP", "label": "비상정지", "level": "ERROR", "recovery_required": True},
}

VALID_COMMANDS = {
    "PAUSE",
    "RESUME",
    "CANCEL",
    "ESTOP",
    "ESTOP_RELEASE",
    "HOME_RETURN",
    "ESTOP_RELEASE_HOME",
    "RECOVER",
}


@dataclass
class BridgeSnapshot:
    connected: bool = False
    last_monitor_age_sec: float = 999.0
    fsm: str = "IDLE"
    hw_code: int = 1
    hw_state: str = "STANDBY"
    hw_label: str = "정상 대기"
    level: str = "OK"
    waiting_recovery: bool = False
    recovery_required: bool = False
    countdown: int = 0
    last_log: str = ""
    last_log_level: str = "INFO"


class BartenderAdminControlBridge(Node):
    def __init__(self):
        super().__init__("bartender_admin_control_bridge")

        self._last_monitor_time = 0.0
        self._snapshot = BridgeSnapshot()
        self._logs: Deque[Dict[str, str]] = deque(maxlen=80)
        self._prev_hw_code = 1
        self._prev_waiting_recovery = False
        self._last_admin_command = ""
        self._last_admin_command_time = 0.0

        # UI command input
        self.create_subscription(String, ADMIN_CONTROL_TOPIC, self._admin_command_cb, 10)

        # Robot/control output
        self.task_pub = self.create_publisher(String, TASK_CONTROL_TOPIC, 10)
        self.estop_pub = self.create_publisher(Bool, ESTOP_TOPIC, 10)
        self.recovery_pub = self.create_publisher(String, RECOVERY_TOPIC, 10)

        # Robot monitor input and UI status output
        self.create_subscription(String, MONITOR_STATUS_TOPIC, self._monitor_status_cb, 10)
        self.bridge_status_pub = self.create_publisher(String, BRIDGE_STATUS_TOPIC, 10)

        self.create_timer(0.3, self._status_timer_cb)

        self._add_log("INFO", "admin control bridge started")
        self._add_log("INFO", f"subscribing {ADMIN_CONTROL_TOPIC}, {MONITOR_STATUS_TOPIC}")
        self._add_log("INFO", f"publishing {TASK_CONTROL_TOPIC}, {ESTOP_TOPIC}, {RECOVERY_TOPIC}, {BRIDGE_STATUS_TOPIC}")

    # ───────────────────────────────────────────────
    # UI command handling
    # ───────────────────────────────────────────────
    def _admin_command_cb(self, msg: String):
        command = msg.data.strip().upper()
        if command not in VALID_COMMANDS:
            self._add_log("WARN", f"unknown admin command ignored: {command}")
            return

        now = time.time()
        if command == self._last_admin_command and now - self._last_admin_command_time < 0.8:
            self._add_log("WARN", f"duplicate admin command ignored: {command}")
            return
        self._last_admin_command = command
        self._last_admin_command_time = now

        if command in ("PAUSE", "RESUME", "CANCEL", "HOME_RETURN", "ESTOP_RELEASE_HOME"):
            out = String()
            out.data = command
            self.task_pub.publish(out)
            self._add_log("OK", f"task control published: {command}")
            return

        if command == "ESTOP":
            out = Bool()
            out.data = True
            self.estop_pub.publish(out)
            self._add_log("ERROR", "emergency stop published: True")
            return

        if command == "ESTOP_RELEASE":
            out = Bool()
            out.data = False
            self.estop_pub.publish(out)
            self._add_log("OK", "emergency stop release published: False")
            return

        if command == "RECOVER":
            out = String()
            out.data = "RECOVER"
            self.recovery_pub.publish(out)
            self._add_log("OK", "recovery command published: RECOVER")
            return

    # ───────────────────────────────────────────────
    # Robot monitor status handling
    # ───────────────────────────────────────────────
    def _monitor_status_cb(self, msg: String):
        self._last_monitor_time = time.time()
        try:
            data = json.loads(msg.data)
        except Exception as exc:
            self._add_log("ERROR", f"monitor JSON parse failed: {exc}")
            return

        hw_code = int(data.get("hw_code", 1))
        hw = HW_MAP.get(hw_code, {"state": "UNKNOWN", "label": "알 수 없음", "level": "WARN", "recovery_required": True})
        waiting_recovery = bool(data.get("waiting_recovery", False))
        log_msg = str(data.get("log", "") or "")
        log_level = str(data.get("log_level", "INFO") or "INFO").upper()

        self._snapshot.connected = True
        self._snapshot.last_monitor_age_sec = 0.0
        self._snapshot.fsm = str(data.get("fsm", "IDLE"))
        self._snapshot.hw_code = hw_code
        self._snapshot.hw_state = hw["state"]
        self._snapshot.hw_label = hw["label"]
        self._snapshot.level = hw["level"]
        self._snapshot.waiting_recovery = waiting_recovery
        self._snapshot.recovery_required = bool(hw["recovery_required"] or waiting_recovery)
        self._snapshot.countdown = int(data.get("countdown", 0) or 0)
        self._snapshot.last_log = log_msg
        self._snapshot.last_log_level = log_level

        # status transition logs
        if hw_code != self._prev_hw_code:
            if hw_code == 5:
                self._add_log("WARN", "안전정지(PROT_STOP) 감지")
            elif hw_code == 6:
                self._add_log("ERROR", "비상정지(EMRG_STOP) 감지")
            elif hw_code == 3:
                self._add_log("ERROR", "서보 꺼짐(SAFE_OFF) 감지")
            elif self._prev_hw_code in (3, 5, 6) and hw_code == 1:
                self._add_log("OK", f"하드웨어 복구 완료: {self._prev_hw_code} -> STANDBY")
            else:
                self._add_log("INFO", f"hardware state changed: {self._prev_hw_code} -> {hw_code}")
            self._prev_hw_code = hw_code

        if waiting_recovery and not self._prev_waiting_recovery:
            self._add_log("WARN", "Recovery 대기 상태 진입 — 관리자 UI에서 복구 버튼 필요")
        if not waiting_recovery and self._prev_waiting_recovery:
            self._add_log("OK", "Recovery 대기 상태 해제")
        self._prev_waiting_recovery = waiting_recovery

        if log_msg:
            self._add_log(log_level, log_msg)

        self._publish_bridge_status()

    # ───────────────────────────────────────────────
    # Periodic status
    # ───────────────────────────────────────────────
    def _status_timer_cb(self):
        if self._last_monitor_time > 0:
            age = time.time() - self._last_monitor_time
            self._snapshot.last_monitor_age_sec = round(age, 2)
            connected = age < 3.0
            if self._snapshot.connected and not connected:
                self._add_log("WARN", "robot_monitor_status 수신 끊김")
            self._snapshot.connected = connected
        else:
            self._snapshot.connected = False
            self._snapshot.last_monitor_age_sec = 999.0

        self._publish_bridge_status()

    def _publish_bridge_status(self):
        payload: Dict[str, Any] = asdict(self._snapshot)
        payload["logs"] = list(self._logs)
        out = String()
        out.data = json.dumps(payload, ensure_ascii=False)
        self.bridge_status_pub.publish(out)

    def _add_log(self, level: str, message: str):
        stamp = time.strftime("%H:%M:%S")
        level = level.upper()
        self._logs.appendleft({"time": stamp, "level": level, "message": str(message)})
        log_line = f"[{level}] {message}"
        if level in ("ERROR", "ERR"):
            self.get_logger().error(log_line)
        elif level in ("WARN", "WARNING"):
            self.get_logger().warn(log_line)
        else:
            self.get_logger().info(log_line)


def main(args=None):
    rclpy.init(args=args)
    node = BartenderAdminControlBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
