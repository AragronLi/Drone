"""
gcs_main.py — 四旋翼无人机地面站上位机

基于 MAVLink v2 协议, 通过 ATK LoRa 数传模块与飞控通信。
依赖: pyserial, pymavlink, matplotlib

使用方法:
    python gcs_main.py
"""

import tkinter as tk
from tkinter import ttk, messagebox
import math
import time

from pymavlink import mavutil

from gcs_mavlink import GCSMavLink
from gcs_dashboard import DashboardFrame, set_mavutil_ref, AttitudeIndicator
from gcs_plot import TelemetryPlot
from gcs_control import ControlPanel


class GroundStationApp:
    """地面站主应用"""

    UPDATE_INTERVAL_MS = 50  # GUI 更新周期

    def __init__(self):
        self._gcs = GCSMavLink()

        self._root = tk.Tk()
        self._root.title("Drone GCS — 四旋翼无人机地面站")
        self._root.geometry("1280x720")
        self._root.configure(bg="#2c3e50")

        # 通知 dashboard 模块 mavutil 引用
        set_mavutil_ref(mavutil)

        self._build_ui()
        self._root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._root.after(self.UPDATE_INTERVAL_MS, self._update_loop)

    def _build_ui(self):
        """构建主界面"""
        # --- 顶部: 连接栏 ---
        topbar = ttk.Frame(self._root)
        topbar.pack(fill=tk.X, padx=5, pady=5)

        ttk.Label(topbar, text="串口:", font=("Arial", 10)).pack(side=tk.LEFT, padx=2)

        self._port_var = tk.StringVar()
        self._port_combo = ttk.Combobox(topbar, textvariable=self._port_var,
                                        width=15, state="readonly")
        self._port_combo.pack(side=tk.LEFT, padx=2)

        ttk.Button(topbar, text="刷新", width=6,
                   command=self._refresh_ports).pack(side=tk.LEFT, padx=2)

        ttk.Label(topbar, text="波特率:", font=("Arial", 10)).pack(side=tk.LEFT, padx=(10, 2))

        self._baud_var = tk.StringVar(value="115200")
        baud_combo = ttk.Combobox(topbar, textvariable=self._baud_var,
                                  width=10, state="readonly",
                                  values=["9600", "38400", "57600", "115200", "460800"])
        baud_combo.pack(side=tk.LEFT, padx=2)

        self._connect_btn = ttk.Button(topbar, text="连接", width=8,
                                       command=self._toggle_connect)
        self._connect_btn.pack(side=tk.LEFT, padx=10)

        # 连接状态
        self._status_label = ttk.Label(topbar, text="未连接",
                                       font=("Arial", 10, "bold"),
                                       foreground="#e74c3c")
        self._status_label.pack(side=tk.LEFT, padx=5)

        self._rx_label = ttk.Label(topbar, text="RX: 0",
                                   font=("Consolas", 9), foreground="#7f8c8d")
        self._rx_label.pack(side=tk.RIGHT, padx=5)

        # --- 主体: 三栏布局 ---
        main_frame = ttk.Frame(self._root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=2)

        # 左面板: 仪表盘
        self._dashboard = DashboardFrame(main_frame)
        self._dashboard.pack(side=tk.LEFT, fill=tk.Y, padx=2)

        # 中央: 波形图
        self._plot = TelemetryPlot(main_frame)
        self._plot.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=2)

        # 右面板: 控制
        self._control = ControlPanel(main_frame, self._gcs)
        self._control.pack(side=tk.RIGHT, fill=tk.Y, padx=2)

        # 初始刷新串口列表
        self._refresh_ports()

    # ---- 串口连接管理 ----

    def _refresh_ports(self):
        """刷新可用串口列表"""
        ports = self._gcs.list_ports()
        port_names = [p[0] for p in ports]
        self._port_combo["values"] = port_names
        if port_names and not self._port_var.get():
            self._port_var.set(port_names[0])

    def _toggle_connect(self):
        """连接/断开切换"""
        if self._gcs.connected:
            self._gcs.disconnect()
            self._connect_btn.config(text="连接")
            self._status_label.config(text="未连接", foreground="#e74c3c")
            self._dashboard.set_connected(False)
            self._plot.reset()
        else:
            port = self._port_var.get()
            if not port:
                messagebox.showwarning("提示", "请选择串口")
                return
            try:
                baud = int(self._baud_var.get())
            except ValueError:
                baud = 115200

            if self._gcs.connect(port, baud):
                self._connect_btn.config(text="断开")
                self._status_label.config(text="已连接", foreground="#2ecc71")
                self._dashboard.set_connected(True)
            else:
                messagebox.showerror("错误", f"无法打开串口 {port}")

    # ---- 主更新循环 ----

    def _update_loop(self):
        """定时器回调: 批量接收消息并更新 UI"""
        if self._gcs.connected:
            try:
                msgs = self._gcs.get_messages_batch(max_count=50)
                if msgs:
                    self._dashboard.update_from_messages(msgs)
                    # 提取姿态数据用于波形
                    for msg in msgs:
                        msg_type = msg.get_type()
                        if msg_type == "ATTITUDE":
                            roll = math.degrees(msg.roll)
                            pitch = math.degrees(msg.pitch)
                            yaw = math.degrees(msg.yaw)
                            # 在此暂时用 roll/pitch/yaw 填充波形
                            # 后续可扩展为批量更新
                            self._plot.add_sample(roll, pitch, yaw,
                                                  0.0, 0.0, math.degrees(msg.rollspeed))
                        elif msg_type == "ALTITUDE":
                            # 更新高度波形 (延迟到下次 ATTITUDE 同步写入)
                            pass

                    self._plot.refresh()

                # 更新统计
                self._rx_label.config(text=f"RX: {self._gcs.packets_rx}")
            except Exception as e:
                print(f"[GCS] UI 更新异常: {e}")

        self._root.after(self.UPDATE_INTERVAL_MS, self._update_loop)

    def _on_close(self):
        """窗口关闭回调"""
        if self._gcs.connected:
            self._gcs.disconnect()
        self._root.destroy()

    def run(self):
        self._root.mainloop()


if __name__ == "__main__":
    app = GroundStationApp()
    app.run()
