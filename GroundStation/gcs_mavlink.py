"""
gcs_mavlink.py — MAVLink 串口通信管理

封装 pymavlink 的 mavutil 连接。
提供非阻塞消息接收 (线程 + queue) 和命令发送接口。
"""

import threading
import queue
import time
import serial.tools.list_ports

from pymavlink import mavutil


class GCSMavLink:
    """MAVLink 地面站通信管理器"""

    MAVLINK_SYS_ID = 255       # 地面站系统 ID
    MAVLINK_COMP_ID = 190      # 地面站组件 ID (MAV_COMP_ID_MISSIONPLANNER)

    def __init__(self):
        self._conn = None
        self._rx_thread = None
        self._running = False
        self._msg_queue = queue.Queue(maxsize=500)
        self._connected = False
        self._port = ""
        self._baudrate = 115200
        self._packets_rx = 0
        self._bytes_rx = 0
        self._last_heartbeat = 0

    # ---- 连接管理 ----

    @staticmethod
    def list_ports():
        """列出可用串口"""
        ports = serial.tools.list_ports.comports()
        return [(p.device, p.description) for p in ports]

    def connect(self, port: str, baudrate: int = 115200) -> bool:
        """打开 MAVLink 串口连接"""
        try:
            self._conn = mavutil.mavlink_connection(
                device=port,
                baud=baudrate,
                source_system=self.MAVLINK_SYS_ID,
                source_component=self.MAVLINK_COMP_ID,
            )
            self._port = port
            self._baudrate = baudrate
            self._connected = True
            self._running = True
            self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self._rx_thread.start()
            return True
        except Exception as e:
            print(f"[GCS] 连接失败: {e}")
            self._connected = False
            return False

    def disconnect(self):
        """关闭连接"""
        self._running = False
        self._connected = False
        if self._conn:
            try:
                self._conn.close()
            except Exception:
                pass
            self._conn = None

    # ---- 消息接收 ----

    def _rx_loop(self):
        """后台线程: 持续读取 MAVLink 消息并放入队列"""
        while self._running:
            try:
                msg = self._conn.recv_msg()
                if msg is not None:
                    self._packets_rx += 1
                    try:
                        self._msg_queue.put_nowait(msg)
                    except queue.Full:
                        # 队列满时丢弃最旧的消息
                        try:
                            self._msg_queue.get_nowait()
                        except queue.Empty:
                            pass
                        self._msg_queue.put_nowait(msg)
            except Exception as e:
                if self._running:
                    print(f"[GCS] 接收错误: {e}")
                    time.sleep(0.5)
                else:
                    break

    def get_message(self, timeout: float = 0.05):
        """非阻塞获取一条消息, 无消息返回 None"""
        try:
            return self._msg_queue.get_nowait()
        except queue.Empty:
            return None

    def get_messages_batch(self, max_count: int = 50):
        """批量获取消息"""
        msgs = []
        for _ in range(max_count):
            msg = self.get_message(timeout=0)
            if msg is None:
                break
            msgs.append(msg)
        return msgs

    # ---- 命令发送 ----

    def send_heartbeat(self):
        """发送地面站心跳"""
        if self._conn:
            self._conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                0, 0, mavutil.mavlink.MAV_STATE_ACTIVE
            )

    def send_arm(self, arm: bool = True):
        """发送 ARM/DISARM 命令"""
        if self._conn:
            self._conn.mav.command_long_send(
                target_system=1,
                target_component=1,
                command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                confirmation=0,
                param1=1.0 if arm else 0.0,
                param2=0, param3=0, param4=0,
                param5=0, param6=0, param7=0
            )

    def send_set_mode(self, custom_mode: int):
        """发送飞行模式切换命令
        custom_mode: 0=STABILIZE, 1=ALTHOLD, 2=AUTO
        """
        if self._conn:
            self._conn.mav.set_mode_send(
                target_system=1,
                base_mode=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                custom_mode=custom_mode
            )

    # ---- 状态属性 ----

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def port(self) -> str:
        return self._port

    @property
    def baudrate(self) -> int:
        return self._baudrate

    @property
    def packets_rx(self) -> int:
        return self._packets_rx

    @property
    def conn(self):
        """直接访问底层 mavutil 连接对象 (用于高级操作)"""
        return self._conn
