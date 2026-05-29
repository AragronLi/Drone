"""
gcs_plot.py — 实时遥测波形绘制

使用 matplotlib 嵌入 tkinter, 6 通道滑动窗口显示。
"""

import collections
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import tkinter as tk
from tkinter import ttk


class TelemetryPlot(ttk.Frame):
    """实时遥测波形图"""

    CHANNELS = [
        ("Roll (°)",      "blue"),
        ("Pitch (°)",     "red"),
        ("Yaw (°)",       "green"),
        ("Altitude (m)",  "purple"),
        ("Battery (V)",   "orange"),
        ("GyroX (dps)",   "brown"),
    ]
    WINDOW_SEC = 30
    MAX_POINTS = 600  # ~20Hz * 30s

    def __init__(self, parent, **kwargs):
        super().__init__(parent, **kwargs)
        self._data = {name: collections.deque(maxlen=self.MAX_POINTS)
                      for name, _ in self.CHANNELS}
        self._time = collections.deque(maxlen=self.MAX_POINTS)
        self._tick = 0

        fig = Figure(figsize=(8, 4.5), dpi=80, facecolor="#1a1a1a")
        self._axes = fig.subplots(3, 2, sharex=True)
        fig.subplots_adjust(hspace=0.35, wspace=0.3, left=0.07, right=0.98, top=0.95, bottom=0.08)

        for ax, (name, color) in zip(self._axes.flat, self.CHANNELS):
            ax.set_facecolor("#1a1a1a")
            ax.tick_params(colors="white", labelsize=7)
            ax.set_title(name, color=color, fontsize=8, fontweight="bold")
            ax.set_xlim(0, self.WINDOW_SEC)
            ax.grid(True, alpha=0.3, color="gray")

        self._lines = {
            name: ax.plot([], [], color=color, linewidth=1.0)[0]
            for ax, (name, color) in zip(self._axes.flat, self.CHANNELS)
        }

        self._canvas = FigureCanvasTkAgg(fig, master=self)
        self._canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self._fig = fig

    def add_sample(self, roll, pitch, yaw, altitude, battery_v, gyro_x):
        """添加一组数据点"""
        self._tick += 1
        t = self._tick * 0.05  # 假定 ~20Hz 更新
        self._time.append(t)

        values = [roll, pitch, yaw, altitude, battery_v, gyro_x]
        for (name, _), val in zip(self.CHANNELS, values):
            self._data[name].append(val)

        # 滑动窗口
        t_min = max(0, t - self.WINDOW_SEC)
        for ax in self._axes.flat:
            ax.set_xlim(t_min, t_min + self.WINDOW_SEC)

        for name, _ in self.CHANNELS:
            self._lines[name].set_data(list(self._time), list(self._data[name]))

        # 自适应 Y 轴
        for ax, (name, _) in zip(self._axes.flat, self.CHANNELS):
            y_data = list(self._data[name])
            if len(y_data) > 1:
                y_min, y_max = min(y_data), max(y_data)
                margin = max(0.1, (y_max - y_min) * 0.15)
                ax.set_ylim(y_min - margin, y_max + margin)

    def refresh(self):
        """刷新画布"""
        try:
            self._canvas.draw_idle()
        except Exception:
            pass

    def reset(self):
        """重置数据"""
        for name, _ in self.CHANNELS:
            self._data[name].clear()
        self._time.clear()
        self._tick = 0
