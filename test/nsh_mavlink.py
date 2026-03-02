#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import re
import sys
import threading
import time

import rospy
from mavros_msgs.msg import Mavlink as RosMavlink
from mavros_msgs.msg import EstimatorStatus
from pymavlink import mavutil
import mavros.mavlink as mavros_mavlink

class NshBridge:
    def __init__(self):
        ns = rospy.get_param("~ns", "/uav1")
        self.pub = rospy.Publisher(f"{ns}/mavlink/to", RosMavlink, queue_size=10)
        self.sub = rospy.Subscriber(f"{ns}/mavlink/from", RosMavlink, self.rx_cb, queue_size=100)
        self.est_sub = rospy.Subscriber(f"{ns}/mavros/estimator_status", EstimatorStatus, self.estimator_cb, queue_size=20)

        # 用于生成 outbound MAVLink 报文
        self.tx = mavutil.mavlink.MAVLink(None, srcSystem=255, srcComponent=190)
        # 用于解析 inbound MAVLink 报文
        self.rx = mavutil.mavlink.MAVLink(None)

        self._ansi_re = re.compile(r"\x1b\[[0-9;?]*[A-Za-z]")
        self._ctrl_re = re.compile(r"[^\x09\x0a\x0d\x20-\x7e]")
        self._cond = threading.Condition()
        self._buf = ""
        self._est_updates = 0
        self._last_est_wall = 0.0

    def send_nsh(self, cmd: str):
        if not cmd.endswith("\n"):
            cmd += "\n"
        b = cmd.encode("utf-8")
        if len(b) > 70:
            raise ValueError("NSH 单条命令最多 70 字节")

        data = list(b) + [0] * (70 - len(b))
        flags = (
            mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND |
            mavutil.mavlink.SERIAL_CONTROL_FLAG_MULTI
        )

        m = self.tx.serial_control_encode(
            mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL,  # device=10
            flags,                                     # flags
            0,                                         # timeout(ms)
            0,                                         # baudrate(对shell可为0)
            len(b),                                    # count
            data                                       # data[70]
        )
        m.pack(self.tx)  # 计算 seq/crc
        ros_msg = mavros_mavlink.convert_to_rosmsg(m)
        self.pub.publish(ros_msg)

    def estimator_cb(self, _msg: EstimatorStatus):
        with self._cond:
            self._est_updates += 1
            self._last_est_wall = time.monotonic()
            self._cond.notify_all()

    def _sanitize(self, text: str) -> str:
        text = self._ansi_re.sub("", text)
        text = self._ctrl_re.sub("", text)
        return text.replace("\r", "")

    def rx_cb(self, ros_msg: RosMavlink):
        try:
            raw = mavros_mavlink.convert_to_bytes(ros_msg)
            msgs = self.rx.parse_buffer(raw) or []
            for m in msgs:
                if m.get_msgId() != mavutil.mavlink.MAVLINK_MSG_ID_SERIAL_CONTROL:
                    continue
                if getattr(m, "device", -1) != mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL:
                    continue
                out = self._sanitize(bytes(m.data[:m.count]).decode("utf-8", errors="ignore"))
                if out:
                    rospy.loginfo("NSH: %s", out.rstrip())
                    with self._cond:
                        self._buf += out
                        self._cond.notify_all()
        except Exception as e:
            rospy.logwarn("parse error: %s", e)

    def run_cmd(self, cmd: str, timeout: float = 8.0) -> str:
        with self._cond:
            self._buf = ""
        self.send_nsh(cmd)

        deadline = time.monotonic() + timeout
        with self._cond:
            while time.monotonic() < deadline:
                if "nsh>" in self._buf or "pxh>" in self._buf:
                    out = self._buf
                    self._buf = ""
                    return out
                remain = max(0.0, deadline - time.monotonic())
                self._cond.wait(timeout=remain)
            out = self._buf
            self._buf = ""
            raise TimeoutError(f"timeout waiting shell prompt after cmd: {cmd}")

    def wait_estimator_update(self, prev_count: int, timeout: float = 4.0) -> bool:
        deadline = time.monotonic() + timeout
        with self._cond:
            while time.monotonic() < deadline:
                if self._est_updates > prev_count:
                    return True
                remain = max(0.0, deadline - time.monotonic())
                self._cond.wait(timeout=remain)
        return False

    def estimator_count(self) -> int:
        with self._cond:
            return self._est_updates

def ekf2_running(status_output: str) -> bool:
    s = status_output.lower()
    if "not running" in s:
        return False
    if "unknown command" in s or "invalid command" in s:
        return False
    running_markers = (
        "available instances:",
        "ekf2:0 ekf dt:",
        "ekf2:1 ekf dt:",
        "ekf2:2 ekf dt:",
    )
    return any(marker in s for marker in running_markers)

def wait_no_estimator_update(bridge: NshBridge, quiet_window: float = 1.2, timeout: float = 4.0) -> bool:
    deadline = time.monotonic() + timeout
    last = bridge.estimator_count()
    last_change = time.monotonic()
    while time.monotonic() < deadline:
        time.sleep(0.1)
        curr = bridge.estimator_count()
        now = time.monotonic()
        if curr != last:
            last = curr
            last_change = now
        if now - last_change >= quiet_window:
            return True
    return False

if __name__ == "__main__":
    rospy.init_node("ekf2_restart_test")
    bridge = NshBridge()
    rospy.sleep(1.5)

    success = True
    try:
        rospy.loginfo("Step 1/4: baseline check")
        status_before = bridge.run_cmd("ekf2 status")
        rospy.loginfo("ekf2 status(before):\n%s", status_before.strip())
        if not ekf2_running(status_before):
            rospy.logerr("FAIL: ekf2 is not running before restart")
            success = False

        rospy.loginfo("Step 2/4: stop ekf2")
        bridge.run_cmd("ekf2 stop")
        time.sleep(0.8)

        rospy.loginfo("Step 3/4: verify ekf2 stopped")
        status_stopped = bridge.run_cmd("ekf2 status")
        rospy.loginfo("ekf2 status(stopped):\n%s", status_stopped.strip())
        if ekf2_running(status_stopped):
            rospy.logerr("FAIL: ekf2 still running after stop")
            success = False

        if not wait_no_estimator_update(bridge, quiet_window=1.0, timeout=3.0):
            rospy.logwarn("WARN: estimator_status did not become quiet after ekf2 stop (continuing)")

        rospy.loginfo("Step 4/4: start ekf2")
        est_before_start = bridge.estimator_count()
        bridge.run_cmd("ekf2 start")
        time.sleep(1.5)

        status_after = bridge.run_cmd("ekf2 status")
        rospy.loginfo("ekf2 status(after):\n%s", status_after.strip())

        if not ekf2_running(status_after):
            rospy.logerr("FAIL: ekf2 not running after start")
            success = False

        if not bridge.wait_estimator_update(est_before_start, timeout=5.0):
            rospy.logerr("FAIL: /mavros/estimator_status not updated after ekf2 start")
            success = False

    except TimeoutError as e:
        rospy.logerr("FAIL: %s", e)
        success = False
    except Exception as e:
        rospy.logerr("FAIL: unexpected exception: %s", e)
        success = False

    if success:
        rospy.loginfo("PASS: EKF2 restart test completed successfully")
        sys.exit(0)

    rospy.logerr("FAIL: EKF2 restart test failed")
    sys.exit(1)
