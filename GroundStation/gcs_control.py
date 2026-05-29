"""
gcs_control.py — 控制面板

ARM/DISARM 按钮、飞行模式选择、命令日志
"""

import tkinter as tk
from tkinter import ttk


class ControlPanel(ttk.LabelFrame):
    """地面站控制面板"""

    MODE_NAMES = {0: "STABILIZE (自稳)", 1: "ALTHOLD (定高)", 2: "AUTO (航点)"}

    def __init__(self, parent, gcs, **kwargs):
        super().__init__(parent, text="飞行控制", padding=10, **kwargs)
        self._gcs = gcs

        # --- ARM / DISARM ---
        btn_frame = ttk.Frame(self)
        btn_frame.pack(fill=tk.X, pady=5)

        self._arm_btn = tk.Button(
            btn_frame, text="ARM 解锁",
            bg="#e74c3c", fg="white",
            font=("Arial", 14, "bold"),
            height=2, width=10,
            command=self._on_arm_click
        )
        self._arm_btn.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)

        self._disarm_btn = tk.Button(
            btn_frame, text="DISARM 锁定",
            bg="#95a5a6", fg="white",
            font=("Arial", 12),
            height=2, width=10,
            command=self._on_disarm_click
        )
        self._disarm_btn.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)

        # --- 模式选择 ---
        ttk.Separator(self).pack(fill=tk.X, pady=10)

        ttk.Label(self, text="飞行模式:", font=("Arial", 11, "bold")).pack(anchor="w")

        self._mode_var = tk.IntVar(value=0)
        self._mode_btns = {}
        mode_frame = ttk.Frame(self)
        mode_frame.pack(fill=tk.X, pady=5)

        for mode_id, mode_name in self.MODE_NAMES.items():
            rb = ttk.Radiobutton(
                mode_frame, text=mode_name, variable=self._mode_var,
                value=mode_id, command=self._on_mode_change
            )
            rb.pack(anchor="w", pady=3)
            self._mode_btns[mode_id] = rb

        # --- 心跳按钮 ---
        ttk.Separator(self).pack(fill=tk.X, pady=10)

        self._hbeat_btn = ttk.Button(
            self, text="发送心跳", command=self._on_send_heartbeat
        )
        self._hbeat_btn.pack(pady=5)

        # --- 命令日志 ---
        ttk.Separator(self).pack(fill=tk.X, pady=10)

        ttk.Label(self, text="命令日志:", font=("Arial", 10, "bold")).pack(anchor="w")
        self._log = tk.Text(self, height=6, width=35, bg="#1a1a1a", fg="#2ecc71",
                            font=("Consolas", 9), state=tk.DISABLED, relief=tk.SUNKEN)
        self._log.pack(fill=tk.BOTH, expand=True, pady=2)

    def _log_cmd(self, text: str):
        """记录命令日志"""
        self._log.config(state=tk.NORMAL)
        self._log.insert(tk.END, f"[GCS] {text}\n")
        self._log.see(tk.END)
        self._log.config(state=tk.DISABLED)

    def _on_arm_click(self):
        if self._gcs.connected:
            self._gcs.send_arm(True)
            self._log_cmd("ARM 解锁命令已发送")
            self._arm_btn.config(bg="#2ecc71")

    def _on_disarm_click(self):
        if self._gcs.connected:
            self._gcs.send_arm(False)
            self._log_cmd("DISARM 锁定命令已发送")
            self._arm_btn.config(bg="#e74c3c")

    def _on_mode_change(self):
        mode = self._mode_var.get()
        if self._gcs.connected:
            self._gcs.send_set_mode(mode)
            self._log_cmd(f"模式切换: {self.MODE_NAMES.get(mode, 'UNKNOWN')}")

    def _on_send_heartbeat(self):
        if self._gcs.connected:
            self._gcs.send_heartbeat()
            self._log_cmd("心跳已发送")
