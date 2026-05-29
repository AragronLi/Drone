"""
gcs_dashboard.py — 飞控仪表盘面板

包含: 姿态地平仪 (Canvas绘制)、状态指示灯、高度/电池数值显示
"""

import math
import tkinter as tk
from tkinter import ttk


class AttitudeIndicator(tk.Canvas):
    """简易人工地平线指示器 (Canvas 自绘)"""

    def __init__(self, parent, size: int = 220, **kwargs):
        super().__init__(parent, width=size, height=size,
                         bg="#1a1a2e", highlightthickness=0, **kwargs)
        self._size = size
        self._cx = size / 2
        self._cy = size / 2
        self._r = size / 2 - 15
        self._roll = 0.0
        self._pitch = 0.0

    def update_attitude(self, roll_deg: float, pitch_deg: float):
        """更新姿态角 (角度制)"""
        self._roll = roll_deg
        self._pitch = pitch_deg
        self._redraw()

    def _redraw(self):
        self.delete("all")
        cx, cy, r = self._cx, self._cy, self._r
        roll_rad = math.radians(self._roll)

        # 计算地平线偏移 (俯仰)
        pitch_offset = (self._pitch / 45.0) * r  # 45° 对应半径偏移

        # 天空 (蓝色) 和地面 (棕色) 分界
        # 地平线随 roll 旋转
        self.create_oval(cx - r, cy - r, cx + r, cy + r,
                         fill="#87CEEB", outline="#555")
        # 地面半圆 (下半部用棕色覆盖)
        # 使用 arc 绘制: 先画地面, 再画旋转的地平线

        # 简化: 绘制旋转的地平线作为天空/地面分界
        h_length = r * 1.5
        x1 = cx - h_length * math.cos(roll_rad) + pitch_offset * math.sin(roll_rad)
        y1 = cy - h_length * math.sin(roll_rad) - pitch_offset * math.cos(roll_rad)
        x2 = cx + h_length * math.cos(roll_rad) - pitch_offset * math.sin(roll_rad)
        y2 = cy + h_length * math.sin(roll_rad) + pitch_offset * math.cos(roll_rad)

        # 地面填充 (地平线以下, 简化为多边形)
        # 计算地平线上下区域
        self.create_line(x1, y1, x2, y2, fill="#e67e22", width=3)

        # 刻度线
        for angle in range(-40, 41, 10):
            a = math.radians(angle)
            sr = math.sin(roll_rad)
            cr = math.cos(roll_rad)
            offset = (angle / 45.0) * r
            px = cx - r * 0.85 * cr + offset * sr
            py = cy - r * 0.85 * sr - offset * cr
            line_len = 20 if angle % 20 == 0 else 10
            ex = px + line_len * cr
            ey = py + line_len * sr
            self.create_line(px, py, ex, ey, fill="white", width=2)
            if angle % 20 == 0:
                self.create_text(ex + 8 * cr, ey + 8 * sr,
                                 text=str(abs(angle)), fill="white",
                                 font=("Consolas", 7))

        # 中心十字 (飞行器标记)
        self.create_line(cx - 30, cy, cx - 10, cy, fill="white", width=2)
        self.create_line(cx + 10, cy, cx + 30, cy, fill="white", width=2)
        self.create_line(cx, cy - 10, cx, cy + 10, fill="white", width=2)
        self.create_oval(cx - 5, cy - 5, cx + 5, cy + 5,
                         fill="#e74c3c", outline="white", width=1)

        # Roll 角弧线 + 指针 (顶部)
        self.create_arc(cx - 50, 10, cx + 50, 60,
                        start=120, extent=300, style="arc",
                        outline="white", width=2)
        for deg in [-60, -30, 0, 30, 60]:
            a = math.radians(deg - 90)
            tx = cx + 45 * math.cos(a)
            ty = 35 + 45 * math.sin(a)
            self.create_text(tx, ty, text=str(deg), fill="white",
                             font=("Consolas", 7))
        # Roll 指针 (三角)
        tp_y = 14
        self.create_polygon(cx, tp_y, cx - 8, tp_y + 12, cx + 8, tp_y + 12,
                            fill="#e74c3c", outline="white")


class DashboardFrame(ttk.Frame):
    """飞控状态仪表盘"""

    def __init__(self, parent, **kwargs):
        super().__init__(parent, **kwargs)
        self._build()

        # 最新数据缓存
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._altitude = 0.0
        self._battery_v = 0.0
        self._battery_i = 0.0
        self._arm_state = False
        self._flight_mode = 0
        self._heartbeat_age = 0

    def _build(self):
        # --- 姿态地平仪 ---
        self.att_indicator = AttitudeIndicator(self, size=200)
        self.att_indicator.grid(row=0, column=0, rowspan=6, padx=10, pady=10)

        # --- 数值标签 ---
        info_frame = ttk.Frame(self)
        info_frame.grid(row=0, column=1, rowspan=6, padx=10, pady=10, sticky="n")

        labels = [
            ("Roll", "0.0°"),
            ("Pitch", "0.0°"),
            ("Yaw", "0.0°"),
            ("Altitude", "0.00 m"),
            ("Battery", "-- V / -- A"),
            ("Status", "DISARMED"),
            ("Mode", "STABILIZE"),
            ("HBeat", "---"),
        ]

        self._value_labels = {}
        row = 0
        for name, default in labels:
            ttk.Label(info_frame, text=f"{name}:", font=("Consolas", 11, "bold"),
                      foreground="#7f8c8d").grid(row=row, column=0, sticky="w", pady=2)
            lbl = ttk.Label(info_frame, text=default, font=("Consolas", 12))
            lbl.grid(row=row, column=1, sticky="e", padx=(10, 0), pady=2)
            self._value_labels[name] = lbl
            row += 1

        # --- 连接状态指示灯 ---
        self._conn_led = tk.Canvas(self, width=16, height=16,
                                   bg=self.cget("style") or "#f0f0f0",
                                   highlightthickness=0)
        self._conn_led.grid(row=6, column=0, columnspan=2, pady=5)
        self._conn_led.create_oval(2, 2, 14, 14, fill="red", tags="led")

    def set_connected(self, state: bool):
        """更新连接指示灯"""
        color = "#2ecc71" if state else "#e74c3c"
        self._conn_led.itemconfig("led", fill=color)

    def update_from_messages(self, messages: list):
        """从 MAVLink 消息列表更新仪表盘"""
        for msg in messages:
            msg_type = msg.get_type()
            if msg_type == "HEARTBEAT":
                self._arm_state = (msg.base_mode &
                                   mavutil_ref().MAV_MODE_FLAG_SAFETY_ARMED) != 0
                self._flight_mode = msg.custom_mode
                self._heartbeat_age = 0
            elif msg_type == "ATTITUDE":
                self._roll = math.degrees(msg.roll)
                self._pitch = math.degrees(msg.pitch)
                self._yaw = math.degrees(msg.yaw)
                self.att_indicator.update_attitude(self._roll, self._pitch)
            elif msg_type == "ALTITUDE":
                self._altitude = msg.altitude_relative
            elif msg_type == "SYS_STATUS":
                self._battery_v = msg.voltage_battery / 1000.0 if msg.voltage_battery > 0 else 0
                self._battery_i = msg.current_battery / 100.0 if msg.current_battery >= 0 else 0

        # 更新显示
        self._value_labels["Roll"].config(text=f"{self._roll:.1f}°")
        self._value_labels["Pitch"].config(text=f"{self._pitch:.1f}°")
        self._value_labels["Yaw"].config(text=f"{self._yaw:.1f}°")
        self._value_labels["Altitude"].config(text=f"{self._altitude:.2f} m")
        if self._battery_v > 0:
            self._value_labels["Battery"].config(text=f"{self._battery_v:.1f}V / {self._battery_i:.1f}A")
        else:
            self._value_labels["Battery"].config(text="--")

        # 状态
        status_text = "ARMED" if self._arm_state else "DISARMED"
        color = "#e74c3c" if not self._arm_state else "#2ecc71"
        self._value_labels["Status"].config(text=status_text, foreground=color)

        # 飞行模式
        mode_names = {0: "STABILIZE", 1: "ALTHOLD", 2: "AUTO"}
        self._value_labels["Mode"].config(text=mode_names.get(self._flight_mode, f"UNKNOWN({self._flight_mode})"))

        # 心跳
        self._value_labels["HBeat"].config(text=f"{self._heartbeat_age}")


# 模块级引用 (在 gcs_main 中设置)
_g_mavutil = None


def set_mavutil_ref(mod):
    global _g_mavutil
    _g_mavutil = mod


def mavutil_ref():
    return _g_mavutil
