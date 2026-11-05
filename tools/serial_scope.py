#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BeyondFOC 上位机（UART3 / 4,000,000 bps）

* 左侧/顶部：选串口 -> 连接；选模式、给目标（电流 A / 转速 rpm）、点「使能」
* 曲线：三相电流、SVPWM 占空比、转速、母线电压，实时刷新
* 没有硬件可以 `--demo` 看界面

帧格式（和固件 App/protocol.h 一致，小端）
------------------------------------------------
MCU -> PC (type 0x01, len 66)
    A5 5A | 01 | 42 | payload(66) | XOR
    payload = ia ib ic da db dc id iq (8×float32)
              + vbus_mV(u16) + speed_rpm(i16) + status(u16) + seq(u16)
              + raw_a raw_b raw_c (u16×3) + isr_avg isr_max (u16×2)
              + enc_deg(f32) + enc_rpm(i16) + enc_crc(u16) + enc_raw(u32)
              + elec_offset_deg(f32)

PC -> MCU (type 0x10, len 12)
    A5 5A | 10 | 0C | payload(12) | XOR
    payload = enable(u8) mode(u8) iq_mA(i16) speed_rpm(i16) flags(u16) reserved(u32)

    mode: 0=电流闭环 1=开环强拖 2=波形观察 3=转速闭环
    status bit0=使能 bit1=转速环 bit2=编码器有效 bit3=电流零偏校准中
           bit4=ADC校准失败 bit5=电角度标定中 bit6=电角度标定完成 bit7=电角度标定失败
           bit8-11=启动阶段 bit12-14=固件实际模式 bit15=最近200ms收到命令
    flags bit0=电流零偏校准 bit1=编码器角度清零 bit2=电角度零位标定

依赖：pip install pyserial pyqtgraph PyQt5 numpy
"""

import argparse
import math
import struct
import sys
import threading
import time
from collections import deque

import numpy as np
import serial
from serial.tools import list_ports

from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg

# ---------------------------------------------------------------- 协议常量
HEAD0 = 0xA5
HEAD1 = 0x5A

TYPE_SCOPE = 0x01
PAYLOAD_LEN = 136
FRAME_LEN = 4 + PAYLOAD_LEN + 1
# Ia Ib Ic | Da Db Dc | Id Iq | Vbus | rpm | status | seq | raw_a raw_b raw_c
# | isr_avg isr_max | enc_deg | enc_rpm | enc_crc_err | enc_raw(24bit)
# | elec_offset_deg | kp_d ki_d kp_q ki_q | offs_a offs_b offs_c | iq_target id_target
# | speed_kp speed_ki pos_kp pos_kd | speed_target | pos_target
# | test_freq vd vq | ld_x10 lq_x10
UNPACK_FMT = "<8fHhHHHHHHHfhHIfffffHHHfffffffffffHH"

TYPE_CMD = 0x10
CMD_PAYLOAD = 46
CMD_FMT = "<BBhhHiffffhffffBff"   # enable mode iq_mA rpm flags pos·0.01 kp_d ki_d kp_q ki_q id_mA speed_kp speed_ki pos_kp pos_kd test_type test_amp test_freq
FLAG_CAL_CURRENT = 0x0001
FLAG_RESET_ANGLE = 0x0002
FLAG_CAL_ANGLE = 0x0004
FLAG_TUNE_PI = 0x0008
FLAG_TUNE_SPEED_PI = 0x0020
FLAG_TUNE_POS_PI = 0x0040

# ADC 满量程电流：3.3V / (5mΩ × 50) = 13.2A，界面按 90% 给上限
CURRENT_FULL_SCALE_A = 13.2
CURRENT_UI_MAX_A = CURRENT_FULL_SCALE_A * 0.9

# 每张图包含哪些通道（选择显示时就按这个整组开关）
PLOT_DEFS = (
    ("cur",   "电流",   ("ia", "ib", "ic")),
    ("duty",  "占空比", ("da", "db", "dc")),
    ("iq",    "iq",     ("iq", "iq_t")),
    ("id",    "id",     ("id", "id_t")),
    ("speed", "转速",   ("speed", "speed_t")),
    ("enc",   "角度",   ("enc_deg", "pos_t")),
    ("vbus",  "母线",   ("vbus",)),
)
PLOT_CHANNELS = {pid: ch for pid, _t, ch in PLOT_DEFS}
PLOT_TITLES = {pid: t for pid, t, _ch in PLOT_DEFS}

SAMPLE_RATE_HZ = 1000.0       # 固件每 1ms 发一帧
MAX_PLOT_POINTS = 1500        # 每次刷新最多画这么多点（超了等间隔抽点）
SMOOTH_WINDOW = 20            # 显示平滑：20 点滑动平均，1kHz 下约 50Hz 低通
DEFAULT_REFRESH_HZ = 25       # 默认刷新率，可调
BAUDRATE = 4_000_000
CMD_PERIOD_MS = 50            # 命令 20Hz 重发，丢一帧也没关系

MODE_CURRENT, MODE_OPENLOOP, MODE_VIEW, MODE_ENCODER, MODE_POSITION = 0, 1, 2, 3, 4
MODE_CHOICES = (
    (MODE_CURRENT, "电流闭环"),
    (MODE_OPENLOOP, "开环强拖"),
    (MODE_ENCODER, "转速闭环"),
    (MODE_POSITION, "位置闭环"),
)
FW_MODE_TEXT = {
    MODE_CURRENT: "电流闭环",
    MODE_OPENLOOP: "开环强拖",
    MODE_VIEW: "波形观察",
    MODE_ENCODER: "转速闭环",
    MODE_POSITION: "位置闭环",
}
STAGE_TEXT = ["上电", "ADC校准中", "TIM2已启动", "串口已挂", "PWM已启动", "采样已启动", "初始化完", "就绪"]


# ---------------------------------------------------------------- 帧解析
class FrameParser:
    """把字节流切成帧；帧头错位/校验错就丢 2 字节重同步。"""

    def __init__(self):
        self.buf = bytearray()
        self.bad = 0
        self.last_type = None
        self.last_len = None

    def feed(self, data: bytes):
        self.buf += data
        out = []
        header = bytes((HEAD0, HEAD1))

        while True:
            i = self.buf.find(header)
            if i < 0:
                # 保留可能的半个帧头（0xA5 在末尾）
                if self.buf and self.buf[-1] == HEAD0:
                    self.buf = bytearray(self.buf[-1:])
                else:
                    self.buf.clear()
                break
            if i > 0:
                del self.buf[:i]

            if len(self.buf) < 4:
                break

            typ = self.buf[2]
            ln = self.buf[3]
            self.last_type = typ
            self.last_len = ln

            if typ != TYPE_SCOPE or ln != PAYLOAD_LEN:
                self.bad += 1
                del self.buf[:2]
                continue

            if len(self.buf) < FRAME_LEN:
                break

            frame = bytes(self.buf[:FRAME_LEN])
            chk = 0
            for b in frame[2:4 + ln]:
                chk ^= b
            if chk != frame[-1]:
                self.bad += 1
                del self.buf[:2]
                continue

            out.append(struct.unpack(UNPACK_FMT, frame[4:4 + ln]))
            del self.buf[:FRAME_LEN]

        return out


def build_cmd(enable, mode, iq_ma, speed_rpm, flags=0, position_cdeg=0,
              kp_d=0.0, ki_d=0.0, kp_q=0.0, ki_q=0.0, id_ma=0,
              speed_kp=0.0, speed_ki=0.0, pos_kp=0.0, pos_kd=0.0,
              test_type=0, test_amp=0.0, test_freq=0.0):
    """PC -> MCU 一帧命令"""
    payload = struct.pack(CMD_FMT, enable, mode, int(iq_ma), int(speed_rpm), flags,
                          int(position_cdeg), float(kp_d), float(ki_d), float(kp_q), float(ki_q),
                          int(id_ma), float(speed_kp), float(speed_ki), float(pos_kp), float(pos_kd),
                          int(test_type), float(test_amp), float(test_freq))
    body = bytes((TYPE_CMD, len(payload))) + payload
    chk = 0
    for b in body:
        chk ^= b
    return bytes((HEAD0, HEAD1)) + body + bytes((chk,))


# ---------------------------------------------------------------- 数据缓存
class DataStore:
    """环形缓存。

    关键优化：`snapshot()` 只把'勾选了'的通道转成 numpy 数组，并且按 max_points
    等间隔抽点。9 条曲线 × 6000 点全量转换/绘制是 CPU 大户，按需转换能省一大截。
    """

    KEYS = ("ia", "ib", "ic", "da", "db", "dc", "speed", "speed_t", "iq", "iq_t", "id", "id_t", "vbus", "enc_deg", "pos_t", "test_freq")

    def __init__(self, maxlen=30000):
        self.lock = threading.Lock()
        self.chan = {k: deque(maxlen=maxlen) for k in self.KEYS}
        self.frames = 0
        self.lost = 0
        self.bad = 0
        self.rx_bytes = 0
        self.last_seq = None
        self.status = 0
        self.raw = (0, 0, 0)
        self.isr = (0, 0)
        self.enc = (0.0, 0, 0, 0)
        self.elec_offset_deg = 0.0
        self.pi = (0.0, 0.0, 0.0, 0.0)      # kp_d, ki_d, kp_q, ki_q（固件回显）
        self.pi_sp = (0.0, 0.0, 0.0, 0.0)   # 转速/位置环 Kp/Ki（固件回显）
        self.offs = (0, 0, 0)               # 固件实际生效的三相电流零偏（ADC 码值）
        self.vd = 0.0
        self.vq = 0.0
        self.ld_x10 = 0
        self.lq_x10 = 0
        self.proto = (None, None)

    def add_bytes(self, n):
        with self.lock:
            self.rx_bytes += n

    def set_proto(self, typ, ln):
        with self.lock:
            self.proto = (typ, ln)

    def clear(self):
        with self.lock:
            for d in self.chan.values():
                d.clear()
            self.frames = 0
            self.lost = 0
            self.bad = 0
            self.rx_bytes = 0
            self.last_seq = None

    def add(self, v):
        (ia, ib, ic, da, db, dc, idd, iq, vbus_mv, rpm,
         status, seq, ra, rb, rc, isr_avg, isr_max,
         enc_deg, enc_rpm, enc_crc, enc_raw, elec_offset_deg,
         kp_d, ki_d, kp_q, ki_q, offs_a, offs_b, offs_c, iq_t, id_t,
         spd_kp, spd_ki, pos_kp, pos_kd, spd_t, pos_t, test_freq, vd, vq,
         ld_x10, lq_x10) = v

        # 固件已经过滤过 NaN/Inf，这里再兜一层，坏帧直接丢
        if not all(math.isfinite(x) for x in (ia, ib, ic, da, db, dc, idd, iq)):
            with self.lock:
                self.bad += 1
            return

        with self.lock:
            c = self.chan
            c["ia"].append(ia)
            c["ib"].append(ib)
            c["ic"].append(ic)
            c["da"].append(da)
            c["db"].append(db)
            c["dc"].append(dc)
            c["speed"].append(rpm)
            c["speed_t"].append(spd_t)
            c["iq"].append(iq)
            c["iq_t"].append(iq_t)
            c["id"].append(idd)
            c["id_t"].append(id_t)
            c["vbus"].append(vbus_mv / 1000.0)
            c["enc_deg"].append(enc_deg)
            c["pos_t"].append(pos_t)
            c["test_freq"].append(test_freq)
            self.raw = (ra, rb, rc)
            self.isr = (isr_avg, isr_max)
            self.enc = (enc_deg, enc_rpm, enc_crc, enc_raw)
            self.elec_offset_deg = elec_offset_deg
            self.pi = (kp_d, ki_d, kp_q, ki_q)
            self.pi_sp = (spd_kp, spd_ki, pos_kp, pos_kd)
            self.offs = (offs_a, offs_b, offs_c)
            self.vd = vd
            self.vq = vq
            self.ld_x10 = ld_x10
            self.lq_x10 = lq_x10
            self.status = status
            self.frames += 1

            if self.last_seq is not None:
                gap = (seq - self.last_seq - 1) & 0xFFFF
                if 0 < gap < 100:
                    self.lost += gap
            self.last_seq = seq

    def snapshot(self, keys, max_points=MAX_PLOT_POINTS, tail=None):
        """只转换 keys 里的通道；点数超过 max_points 就等间隔抽点。

        tail：只取最后 tail 个采样点（配合"时间窗"缩放用）。
              这样窗口拉小时能保持 1ms 的原始分辨率，不会被整体抽点抽没。
        """
        with self.lock:
            total = len(self.chan["ia"])
            if total == 0:
                return None

            n = total
            if tail is not None and 0 < tail < total:
                n = tail

            step = 1 if n <= max_points else int(math.ceil(n / max_points))

            data = {}
            for k in keys:
                d = self.chan[k]
                arr = np.fromiter(d, dtype=np.float32, count=len(d))
                if n < len(arr):
                    arr = arr[-n:]
                data[k] = arr[::step] if step > 1 else arr

            return data, n, step

    def last_values(self):
        """状态栏用：只取最后一个值，不做数组转换。"""
        with self.lock:
            c = self.chan
            return (self.frames, self.lost, self.status, self.raw, self.isr, self.enc,
                    c["iq"][-1] if c["iq"] else float("nan"),
                    c["speed"][-1] if c["speed"] else 0.0,
                    c["speed_t"][-1] if c["speed_t"] else 0.0,
                    c["vbus"][-1] if c["vbus"] else float("nan"),
                    self.elec_offset_deg, self.pi, self.offs,
                    c["iq_t"][-1] if c["iq_t"] else float("nan"),
                    c["id"][-1] if c["id"] else float("nan"),
                    c["id_t"][-1] if c["id_t"] else float("nan"),
                    self.pi_sp)


class SerialReader(QtCore.QThread):
    error = QtCore.pyqtSignal(str)

    def __init__(self, port, baud, store: DataStore):
        super().__init__()
        self.port = port
        self.baud = baud
        self.store = store
        self._run = True
        self._ser = None
        self._wlock = threading.Lock()

    def send(self, data: bytes):
        with self._wlock:
            if self._ser is not None and self._ser.is_open:
                try:
                    self._ser.write(data)
                except Exception:          # noqa: BLE001
                    pass

    def run(self):
        parser = FrameParser()
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.05)
            self._ser.reset_input_buffer()
        except Exception as exc:            # noqa: BLE001
            self.error.emit(f"打开 {self.port} 失败：{exc}")
            return

        try:
            while self._run:
                data = self._ser.read(4096)
                if not data:
                    continue
                self.store.add_bytes(len(data))
                frames = parser.feed(data)
                self.store.set_proto(parser.last_type, parser.last_len)
                for values in frames:
                    self.store.add(values)
        except Exception as exc:            # noqa: BLE001
            if self._run:
                self.error.emit(f"串口读取中断：{exc}")
        finally:
            try:
                self._ser.close()
            except Exception:               # noqa: BLE001
                pass

    def stop(self):
        self._run = False
        self.wait(500)


# ---------------------------------------------------------------- 模拟数据源
class DemoReader(QtCore.QThread):
    """没有硬件时用：1kHz 生成三相正弦电流 + SVPWM 占空比，并响应命令。"""

    error = QtCore.pyqtSignal(str)

    def __init__(self, store: DataStore):
        super().__init__()
        self.store = store
        self._run = True
        self.t0 = time.time()
        self.enable = False
        self.mode = MODE_CURRENT
        self.iq_cmd = 0.0
        self.rpm_cmd = 0.0

    def send(self, data: bytes):
        if len(data) < 4 + CMD_PAYLOAD + 1:
            return
        try:
            enable, mode, iq_ma, rpm, _flags, _res, _kpd, _kid, _kpq, _kiq, _idma, _skp, _ski, _pkp, _pki = \
                struct.unpack(CMD_FMT, data[4:4 + CMD_PAYLOAD])
        except struct.error:
            return
        self.enable = bool(enable)
        self.mode = mode
        self.iq_cmd = iq_ma / 1000.0
        self.rpm_cmd = rpm

    def run(self):
        seq = 0
        dt = 1.0 / SAMPLE_RATE_HZ
        t0 = self.t0
        next_t = time.perf_counter()

        while self._run:
            now = time.perf_counter()
            if now < next_t:
                time.sleep(min(0.002, next_t - now))
                continue

            while self._run and next_t <= now:
                t = next_t - t0
                w = 2 * np.pi * 20.0
                amp = 2.0 if (self.enable and self.mode != MODE_VIEW) else 0.0

                ia = amp * np.sin(w * t)
                ib = amp * np.sin(w * t - 2 * np.pi / 3)
                ic = amp * np.sin(w * t + 2 * np.pi / 3)

                # 波形观察模式：无论使能都输出调制波
                if self.mode == MODE_VIEW or self.enable:
                    da = 0.5 + 0.4 * np.sin(w * t)
                    db = 0.5 + 0.4 * np.sin(w * t - 2 * np.pi / 3)
                    dc = 0.5 + 0.4 * np.sin(w * t + 2 * np.pi / 3)
                else:
                    da = db = dc = 0.5

                rpm = self.rpm_cmd if self.mode in (MODE_OPENLOOP, MODE_ENCODER) and self.enable else 0.0
                status = (1 if self.enable else 0) | (2 if self.mode in (MODE_ENCODER, MODE_VIEW) else 0)

                self.store.add((ia, ib, ic, da, db, dc, 0.0, self.iq_cmd,
                                24000, int(rpm), status, seq, 2048, 2048, 2048,
                                42, 98, (t * 360.0) % 360.0, int(rpm), 0, 0x123456,
                                0.0,
                                0.235, 1162.0, 0.397, 1162.0,
                                2050, 2050, 2050, self.iq_cmd, 0.0,
                                0.0682, 1.35, 80.0, 0.0, rpm, 0.0,
                                0.0, 0.0, 0.0, 0, 0))
                seq = (seq + 1) & 0xFFFF
                next_t += dt
                now = time.perf_counter()

    def stop(self):
        self._run = False
        self.wait(500)


# ---------------------------------------------------------------- 主窗口
COLORS = [(255, 90, 90), (90, 220, 120), (90, 170, 255)]
NAMES = ["A 相", "B 相", "C 相"]


class ScopeWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("BeyondFOC 上位机 - 控制 / 电流 / SVPWM")
        self.resize(1500, 920)

        self.store = DataStore()
        self.reader = None
        self.paused = False
        self._last_frames = 0
        self._last_time = time.time()
        self._connected_at = 0.0
        self.cal_flag = False
        self.angle_zero_flag = False
        self.angle_cal_flag = False
        self._port_text = "未连接"

        pg.setConfigOptions(antialias=False, background="k", foreground="w")

        self.setStyleSheet("""
QMainWindow { background: #1e1f22; }
QWidget { color: #dcdcdc; font-size: 12px; }
QGroupBox {
    border: 1px solid #3a3d42; border-radius: 6px;
    margin-top: 10px; padding: 8px 8px 6px 8px;
}
QGroupBox::title {
    subcontrol-origin: margin; subcontrol-position: top left;
    left: 10px; padding: 0 5px; color: #8fbaff; font-weight: bold;
}
QPushButton {
    background: #2f3237; border: 1px solid #4a4e55; border-radius: 4px;
    padding: 5px 12px; color: #e0e0e0;
}
QPushButton:hover { background: #3a3e45; border-color: #6b7280; }
QPushButton:disabled { color: #6a6a6a; border-color: #33363b; }
QComboBox, QSpinBox, QDoubleSpinBox {
    background: #26282c; border: 1px solid #444850; border-radius: 4px;
    padding: 3px 6px; selection-background-color: #3d6fb5;
}
QComboBox QAbstractItemView { background: #26282c; selection-background-color: #3d6fb5; }
QCheckBox { spacing: 5px; color: #d0d0d0; }
QSlider::groove:horizontal { height: 4px; background: #3a3d42; border-radius: 2px; }
QSlider::handle:horizontal {
    width: 12px; margin: -5px 0; border-radius: 6px; background: #6ea8ff;
}
QStatusBar { background: #232529; }
QStatusBar QLabel { color: #cfcfcf; }
""")

        # ================= 顶部：连接 =================
        row1 = QtWidgets.QHBoxLayout()
        row1.addWidget(QtWidgets.QLabel("串口:"))
        self.cb_port = QtWidgets.QComboBox()
        self.cb_port.setMinimumWidth(230)
        row1.addWidget(self.cb_port)

        self.btn_refresh = QtWidgets.QPushButton("刷新")
        self.btn_refresh.clicked.connect(self.refresh_ports)
        row1.addWidget(self.btn_refresh)

        row1.addWidget(QtWidgets.QLabel("波特率:"))
        self.sp_baud = QtWidgets.QSpinBox()
        self.sp_baud.setRange(9600, 6_000_000)
        self.sp_baud.setValue(BAUDRATE)
        row1.addWidget(self.sp_baud)

        self.btn_connect = QtWidgets.QPushButton("连接")
        self.btn_connect.setCheckable(True)
        self.btn_connect.setMinimumWidth(90)
        self.btn_connect.clicked.connect(self.toggle_connect)
        row1.addWidget(self.btn_connect)

        row1.addSpacing(20)
        self.btn_pause = QtWidgets.QPushButton("暂停")
        self.btn_pause.setCheckable(True)
        self.btn_pause.clicked.connect(self.toggle_pause)
        row1.addWidget(self.btn_pause)

        self.btn_clear = QtWidgets.QPushButton("清空")
        self.btn_clear.clicked.connect(self.store.clear)
        row1.addWidget(self.btn_clear)

        self.chk_auto = QtWidgets.QCheckBox("自动缩放")
        self.chk_auto.setChecked(True)
        row1.addWidget(self.chk_auto)

        row1.addSpacing(12)
        row1.addWidget(QtWidgets.QLabel("刷新:"))
        self.sp_fps = QtWidgets.QSpinBox()
        self.sp_fps.setRange(5, 60)
        self.sp_fps.setValue(DEFAULT_REFRESH_HZ)
        self.sp_fps.setSuffix(" Hz")
        self.sp_fps.setToolTip("画面刷新率。电脑卡就调低，数据本身还是 1kHz")
        self.sp_fps.valueChanged.connect(self._apply_refresh)
        row1.addWidget(self.sp_fps)

        # ISR 占用率单独放一个会变色的标签：状态栏太长会被截断，这个永远看得见
        row1.addSpacing(16)
        self.lbl_isr = QtWidgets.QLabel("ISR --.-/--.-us  --%")
        self.lbl_isr.setMinimumWidth(210)
        self.lbl_isr.setToolTip(
            "固件 20kHz 电流环中断的耗时（avg/max）和 CPU 占用。\n"
            "max 接近 50us 就会把 FreeRTOS 任务饿死 → 心跳灯灭、串口停。\n"
            ">80% 红、>50% 黄、其余绿。"
        )
        self.lbl_isr.setStyleSheet("color:#8f8; font-family:Consolas,monospace")
        row1.addWidget(self.lbl_isr)

        row1.addStretch(1)

        # ================= 控制（左侧竖排） =================
        ctrl = QtWidgets.QGridLayout()
        ctrl.setHorizontalSpacing(8)
        ctrl.setVerticalSpacing(6)

        self.cb_mode = QtWidgets.QComboBox()
        for mode, text in MODE_CHOICES:
            self.cb_mode.addItem(text, mode)
        self.cb_mode.currentIndexChanged.connect(self.on_mode_changed)
        ctrl.addWidget(QtWidgets.QLabel("模式:"), 0, 0)
        ctrl.addWidget(self.cb_mode, 0, 1, 1, 2)

        # iq 目标
        self.lbl_iq = QtWidgets.QLabel("电流目标 (A):")
        ctrl.addWidget(self.lbl_iq, 1, 0)
        self.sp_iq = QtWidgets.QDoubleSpinBox()
        self.sp_iq.setRange(-CURRENT_UI_MAX_A, CURRENT_UI_MAX_A)
        self.sp_iq.setSingleStep(0.1)
        self.sp_iq.setDecimals(2)
        self.sp_iq.setValue(0.0)
        self.sp_iq.setMinimumWidth(90)
        ctrl.addWidget(self.sp_iq, 1, 1)
        self.sl_iq = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.sl_iq.setRange(-int(CURRENT_UI_MAX_A * 100), int(CURRENT_UI_MAX_A * 100))
        self.sl_iq.valueChanged.connect(lambda v: self.sp_iq.setValue(v / 100.0))
        self.sp_iq.valueChanged.connect(lambda v: self._sync_slider(self.sl_iq, int(v * 100)))
        ctrl.addWidget(self.sl_iq, 1, 2)

        # id 目标
        self.lbl_id = QtWidgets.QLabel("id 目标 (A):")
        ctrl.addWidget(self.lbl_id, 2, 0)
        self.sp_id = QtWidgets.QDoubleSpinBox()
        self.sp_id.setRange(-CURRENT_UI_MAX_A, CURRENT_UI_MAX_A)
        self.sp_id.setSingleStep(0.05)
        self.sp_id.setDecimals(2)
        self.sp_id.setValue(0.0)
        self.sp_id.setMinimumWidth(90)
        self.sp_id.setToolTip(
            "电流环 d 轴指令（A）。一般保持 0；\n"
            "做弱磁、参数辨识、或者单独调 d 轴电流环时才给非零。\n"
            "开环强拖模式下不生效。"
        )
        ctrl.addWidget(self.sp_id, 2, 1)
        self.sl_id = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.sl_id.setRange(-int(CURRENT_UI_MAX_A * 100), int(CURRENT_UI_MAX_A * 100))
        self.sl_id.valueChanged.connect(lambda v: self.sp_id.setValue(v / 100.0))
        self.sp_id.valueChanged.connect(lambda v: self._sync_slider(self.sl_id, int(v * 100)))
        ctrl.addWidget(self.sl_id, 2, 2)

        # 转速目标
        self.lbl_rpm = QtWidgets.QLabel("转速目标 (rpm):")
        ctrl.addWidget(self.lbl_rpm, 3, 0)
        self.sp_rpm = QtWidgets.QDoubleSpinBox()
        self.sp_rpm.setRange(-20000.0, 20000.0)
        self.sp_rpm.setSingleStep(50.0)
        self.sp_rpm.setDecimals(0)
        self.sp_rpm.setValue(0.0)
        self.sp_rpm.setMinimumWidth(90)
        ctrl.addWidget(self.sp_rpm, 3, 1)
        self.sl_rpm = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.sl_rpm.setRange(-20000, 20000)
        self.sl_rpm.valueChanged.connect(lambda v: self.sp_rpm.setValue(v))
        self.sp_rpm.valueChanged.connect(lambda v: self._sync_slider(self.sl_rpm, int(v)))
        ctrl.addWidget(self.sl_rpm, 3, 2)

        # 位置目标
        ctrl.addWidget(QtWidgets.QLabel("位置目标 (deg):"), 4, 0)
        self.sp_pos = QtWidgets.QDoubleSpinBox()
        self.sp_pos.setRange(-36000.0, 36000.0)
        self.sp_pos.setSingleStep(1.0)
        self.sp_pos.setDecimals(2)
        self.sp_pos.setValue(0.0)
        self.sp_pos.setMinimumWidth(90)
        self.sp_pos.setToolTip(
            "位置闭环目标，机械角度，单位 deg。\n"
            "支持多圈：可以输入 720、1080 这样的角度。")
        ctrl.addWidget(self.sp_pos, 4, 1)

        # 位置拖动条：±3600°（10 圈），0.1° 分辨率
        self.sl_pos = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.sl_pos.setRange(-36000, 36000)          # x0.1 deg
        self.sl_pos.setToolTip("拖动改位置目标（±3600°）；超出范围请在左边直接输入")
        self.sl_pos.valueChanged.connect(lambda v: self.sp_pos.setValue(v / 10.0))
        self.sp_pos.valueChanged.connect(lambda v: self._sync_slider(self.sl_pos, int(round(v * 10.0))))
        ctrl.addWidget(self.sl_pos, 4, 2)

        # 按钮
        btn_row = QtWidgets.QHBoxLayout()
        btn_row.setSpacing(6)

        self.btn_enable = QtWidgets.QPushButton("使能输出")
        self.btn_enable.setCheckable(True)
        self.btn_enable.setMinimumHeight(30)
        self.btn_enable.setStyleSheet(
            "QPushButton{background:#333;color:#ddd;border:1px solid #666;border-radius:4px}"
            "QPushButton:checked{background:#8b1a1a;color:#fff;border:1px solid #ff5555}"
        )
        btn_row.addWidget(self.btn_enable)

        self.btn_cal = QtWidgets.QPushButton("重新校零")
        self.btn_cal.setToolTip("功率级断电时点一下：把三相电流零点重新标定（固件上电时也会自动做一次）")
        self.btn_cal.clicked.connect(self.request_calibration)
        btn_row.addWidget(self.btn_cal)

        self.btn_angle0 = QtWidgets.QPushButton("角度清零")
        self.btn_angle0.setToolTip("把编码器的连续角度清成 0（不影响当前位置）")
        self.btn_angle0.clicked.connect(self.request_angle_zero)
        btn_row.addWidget(self.btn_angle0)

        self.btn_angle_cal = QtWidgets.QPushButton("电角度标定")
        self.btn_angle_cal.setToolTip(
            "多点电角度标定：给 d 轴电流让转子在 8 个电角度（45° 一步）依次对齐，\n"
            "每个点读一次编码器，最后取矢量平均。约 3 秒，期间转子会小幅转动。\n"
            "请确保电机可以自由对齐、电源限流。")
        self.btn_angle_cal.clicked.connect(self.request_angle_calibration)
        btn_row.addWidget(self.btn_angle_cal)

        ctrl.addLayout(btn_row, 5, 0, 1, 3)

        # ================= 第四行：电流环 PI 在线整定 =================
        row4 = QtWidgets.QGridLayout()
        row4.setHorizontalSpacing(8)
        row4.setVerticalSpacing(6)
        self.chk_tune = QtWidgets.QCheckBox("在线整定电流环 PI")
        self.chk_tune.setToolTip(
            "勾上后，下面 4 个值会随命令一起下发（flags bit3）。\n"
            "固件只改增益、不清积分，所以可以在使能状态下直接调。\n"
            "取消勾选 = 固件恢复到上电时按带宽算出来的默认参数。"
        )
        row4.addWidget(self.chk_tune, 0, 0, 1, 2)

        self.sp_kp_d = self._gain_spin("Kp_d:", 0.0, 10.0, 0.001, 4, 0.468)
        self.sp_ki_d = self._gain_spin("Ki_d:", 0.0, 50000.0, 10.0, 1, 1198.0)
        self.sp_kp_q = self._gain_spin("Kp_q:", 0.0, 10.0, 0.001, 4, 0.267)
        self.sp_ki_q = self._gain_spin("Ki_q:", 0.0, 50000.0, 10.0, 1, 1198.0)
        row4.addWidget(self.sp_kp_d, 1, 0)
        row4.addWidget(self.sp_ki_d, 1, 1)
        row4.addWidget(self.sp_kp_q, 2, 0)
        row4.addWidget(self.sp_ki_q, 2, 1)

        self.btn_pi_read = QtWidgets.QPushButton("读当前值")
        self.btn_pi_read.setToolTip("把固件回显的实际 Kp/Ki 填到上面四个框里")
        self.btn_pi_read.clicked.connect(self.read_pi_from_fw)
        row4.addWidget(self.btn_pi_read, 3, 0, 1, 2)

        # 勾上在线整定时先读一次固件当前值，避免用 UI 里的旧默认值把固件覆盖掉
        self.chk_tune.toggled.connect(self._on_tune_toggled)

        # ================= 第五行：转速 / 位置环 PI 在线整定 =================
        row5 = QtWidgets.QGridLayout()
        row5.setHorizontalSpacing(8)
        row5.setVerticalSpacing(6)
        self.chk_tune_sp = QtWidgets.QCheckBox("在线整定转速 PI / 位置 PD")
        self.chk_tune_sp.setToolTip(
            "勾上后，下面 4 个值随命令下发（flags bit5/bit6）。\n"
            "速度环：Kp/Ki；位置环：Kp/Kd（PD）。\n"
            "取消勾选 = 固件恢复到 app_config.h 里的默认参数。"
        )
        row5.addWidget(self.chk_tune_sp, 0, 0, 1, 2)

        self.sp_spd_kp = self._gain_spin("速度Kp:", 0.0, 10.0, 0.001, 4, 0.0021)
        self.sp_spd_ki = self._gain_spin("速度Ki:", 0.0, 1000.0, 0.01, 3, 0.052)
        self.sp_pos_kp = self._gain_spin("位置Kp:", 0.0, 1000.0, 1.0, 2, 80.0)
        self.sp_pos_kd = self._gain_spin("位置Kd:", 0.0, 1000.0, 0.1, 2, 0.5)
        self.sp_pos_kd.setToolTip(
            "位置环 PD 的 D 项（速度反馈阻尼）：\n"
            "速度指令 = Kp·位置误差 - Kd·实际机械角速度\n"
            "0.5 起步，往大调增加阻尼；太大反而变迟钝。"
        )
        row5.addWidget(self.sp_spd_kp, 1, 0)
        row5.addWidget(self.sp_spd_ki, 1, 1)
        row5.addWidget(self.sp_pos_kp, 2, 0)
        row5.addWidget(self.sp_pos_kd, 2, 1)

        self.btn_pi_sp_read = QtWidgets.QPushButton("读当前值")
        self.btn_pi_sp_read.setToolTip("把固件回显的转速 Kp/Ki、位置 Kp/Kd 填到上面四个框里")
        self.btn_pi_sp_read.clicked.connect(self.read_pi_sp_from_fw)
        row5.addWidget(self.btn_pi_sp_read, 3, 0, 1, 2)

        self.chk_tune_sp.toggled.connect(self._on_tune_sp_toggled)

        panel = QtWidgets.QWidget()
        pv = QtWidgets.QVBoxLayout(panel)
        pv.setContentsMargins(8, 6, 8, 6)
        pv.setSpacing(6)

        tune_lay = QtWidgets.QVBoxLayout()
        tune_lay.setSpacing(4)
        tune_lay.addLayout(row4)
        tune_lay.addLayout(row5)

        pv.addWidget(self._group("控制", ctrl))
        pv.addWidget(self._group("在线整定（电流环 / 转速环 / 位置环）", tune_lay))

        # ================= 测试信号（固件生成正弦） =================
        test_lay = QtWidgets.QGridLayout()
        test_lay.setHorizontalSpacing(8)
        test_lay.setVerticalSpacing(6)

        self.chk_sine = QtWidgets.QCheckBox("正弦测试")
        self.chk_sine.setToolTip(
            "由固件 1kHz 生成正弦参考。\n"
            "中心值 = 左边当前的速度/位置目标；\n"
            "速度正弦单位 rpm，位置正弦单位 deg。")
        test_lay.addWidget(self.chk_sine, 0, 0, 1, 2)

        self.cb_sine = QtWidgets.QComboBox()
        self.cb_sine.addItem("速度正弦", 1)
        self.cb_sine.addItem("位置正弦", 2)
        test_lay.addWidget(QtWidgets.QLabel("对象:"), 1, 0)
        test_lay.addWidget(self.cb_sine, 1, 1)

        self.sp_sine_amp = QtWidgets.QDoubleSpinBox()
        self.sp_sine_amp.setRange(0.0, 5000.0)
        self.sp_sine_amp.setSingleStep(10.0)
        self.sp_sine_amp.setDecimals(1)
        self.sp_sine_amp.setValue(200.0)
        self.sp_sine_amp.setMinimumWidth(100)
        self.sp_sine_amp.setToolTip("幅值：速度正弦 = rpm，位置正弦 = deg")
        test_lay.addWidget(QtWidgets.QLabel("幅值:"), 2, 0)
        test_lay.addWidget(self.sp_sine_amp, 2, 1)

        self.sp_sine_freq = QtWidgets.QDoubleSpinBox()
        self.sp_sine_freq.setRange(0.01, 50.0)
        self.sp_sine_freq.setSingleStep(0.1)
        self.sp_sine_freq.setDecimals(2)
        self.sp_sine_freq.setValue(1.0)
        self.sp_sine_freq.setMinimumWidth(100)
        self.sp_sine_freq.setToolTip("频率 (Hz)")
        test_lay.addWidget(QtWidgets.QLabel("频率 (Hz):"), 3, 0)
        test_lay.addWidget(self.sp_sine_freq, 3, 1)

        pv.addWidget(self._group("测试信号（固件生成正弦）", test_lay))

        # ================= 曲线 =================
        self.pw_cur = pg.PlotWidget(title="三相电流 (A)")
        self.pw_duty = pg.PlotWidget(title="SVPWM 占空比")
        self.pw_iq = pg.PlotWidget(title="iq (A)：反馈 / 目标")
        self.pw_id = pg.PlotWidget(title="id (A)：反馈 / 目标")
        self.pw_speed = pg.PlotWidget(title="转速 (rpm)")
        self.pw_enc = pg.PlotWidget(title="编码器角度 (机械 °，连续累计)")
        self.pw_vbus = pg.PlotWidget(title="母线电压 (V)")

        self.curves_cur, self.curves_duty = [], []
        for i in range(3):
            self.curves_cur.append(self.pw_cur.plot(pen=pg.mkPen(COLORS[i], width=1.6), name=NAMES[i]))
            self.curves_duty.append(self.pw_duty.plot(pen=pg.mkPen(COLORS[i], width=1.6), name=NAMES[i]))
        self.curve_speed = self.pw_speed.plot(pen=pg.mkPen((255, 200, 80), width=1.6), name="转速 rpm")
        self.curve_speed_t = self.pw_speed.plot(
            pen=pg.mkPen((120, 220, 255), width=1.4, style=QtCore.Qt.DashLine), name="转速目标")
        self.curve_iq = self.pw_iq.plot(pen=pg.mkPen((180, 120, 255), width=1.6), name="iq 反馈")
        self.curve_iq_t = self.pw_iq.plot(pen=pg.mkPen((255, 210, 90), width=1.4,
                                                      style=QtCore.Qt.DashLine), name="iq 目标")
        self.curve_id = self.pw_id.plot(pen=pg.mkPen((120, 220, 255), width=1.6), name="id 反馈")
        self.curve_id_t = self.pw_id.plot(pen=pg.mkPen((255, 160, 80), width=1.4,
                                                      style=QtCore.Qt.DashLine), name="id 目标")
        self.curve_enc = self.pw_enc.plot(pen=pg.mkPen((255, 120, 220), width=1.4), name="角度")
        self.curve_pos_t = self.pw_enc.plot(
            pen=pg.mkPen((120, 220, 255), width=1.4, style=QtCore.Qt.DashLine), name="位置目标")
        self.curve_vbus = self.pw_vbus.plot(pen=pg.mkPen((120, 220, 255), width=1.6))

        for pw in (self.pw_cur, self.pw_duty, self.pw_iq, self.pw_id, self.pw_speed, self.pw_enc, self.pw_vbus):
            pw.showGrid(x=True, y=True, alpha=0.25)
            pw.addLegend(offset=(8, 8))
        self.pw_vbus.setLabel("bottom", "时间 (s，0 = 当前)")
        self.pw_duty.setYRange(0.0, 1.0, padding=0.05)

        for pw in (self.pw_duty, self.pw_iq, self.pw_id, self.pw_speed, self.pw_enc, self.pw_vbus):
            pw.setXLink(self.pw_cur)

        # ---------------- 通道勾选（按“图”选择） ----------------
        self.curves = {
            "ia": self.curves_cur[0], "ib": self.curves_cur[1], "ic": self.curves_cur[2],
            "da": self.curves_duty[0], "db": self.curves_duty[1], "dc": self.curves_duty[2],
            "speed": self.curve_speed, "speed_t": self.curve_speed_t,
            "iq": self.curve_iq, "iq_t": self.curve_iq_t,
            "id": self.curve_id, "id_t": self.curve_id_t,
            "vbus": self.curve_vbus, "enc_deg": self.curve_enc, "pos_t": self.curve_pos_t,
        }
        self.plot_of = {
            "ia": self.pw_cur, "ib": self.pw_cur, "ic": self.pw_cur,
            "da": self.pw_duty, "db": self.pw_duty, "dc": self.pw_duty,
            "speed": self.pw_speed, "speed_t": self.pw_speed, "iq": self.pw_iq, "iq_t": self.pw_iq,
            "id": self.pw_id, "id_t": self.pw_id,
            "vbus": self.pw_vbus, "enc_deg": self.pw_enc, "pos_t": self.pw_enc,
        }
        self.plots = {
            "cur": self.pw_cur, "duty": self.pw_duty, "iq": self.pw_iq,
            "id": self.pw_id, "speed": self.pw_speed,
            "enc": self.pw_enc, "vbus": self.pw_vbus,
        }

        # 只画视窗内的点 + 点数多时自动抽点，能省很多 CPU
        for c in self.curves.values():
            c.setClipToView(True)
            c.setDownsampling(auto=True, method="peak")

        row3 = QtWidgets.QGridLayout()
        row3.setHorizontalSpacing(6)
        row3.setVerticalSpacing(4)
        row3.addWidget(QtWidgets.QLabel("显示图:"), 0, 0, 1, 2)
        self.chk_plot = {}
        default_on = {"cur", "duty", "iq", "id", "speed"}
        for i, (pid, text, _chans) in enumerate(PLOT_DEFS):
            cb = QtWidgets.QCheckBox(text)
            cb.setChecked(pid in default_on)
            cb.toggled.connect(self._apply_visibility)
            self.chk_plot[pid] = cb
            row3.addWidget(cb, 1 + i // 3, i % 3)

        btn_row2 = QtWidgets.QHBoxLayout()
        self.chk_smooth = QtWidgets.QCheckBox("电流平滑")
        self.chk_smooth.setToolTip("只作用于显示：20 点滑动平均，不影响固件里的电流环")
        btn_row2.addWidget(self.chk_smooth)
        btn_all = QtWidgets.QPushButton("全选")
        btn_all.clicked.connect(lambda: self._set_plots(True))
        btn_cur = QtWidgets.QPushButton("仅电流")
        btn_cur.clicked.connect(lambda: self._set_plots(False, {"cur"}))
        btn_row2.addWidget(btn_all)
        btn_row2.addWidget(btn_cur)
        btn_row2.addStretch(1)
        row3.addLayout(btn_row2, 4, 0, 1, 3)

        # 这个显示行稍后和"视图缩放"一起放进分组框

        # ================= 视图缩放：纵向倍数 + 横向时间窗 =================
        row_view = QtWidgets.QGridLayout()
        row_view.setHorizontalSpacing(6)
        row_view.setVerticalSpacing(4)

        row_view.addWidget(QtWidgets.QLabel("纵向缩放 ×:"), 0, 0)
        self.sp_zoom = QtWidgets.QDoubleSpinBox()
        self.sp_zoom.setRange(0.1, 50.0)
        self.sp_zoom.setSingleStep(0.5)
        self.sp_zoom.setDecimals(1)
        self.sp_zoom.setValue(1.0)
        self.sp_zoom.setMinimumWidth(78)
        self.sp_zoom.setToolTip(
            "只影响显示，不改固件数据。\n"
            "1.0 = 自动铺满；2.0 = 纵向放大 2 倍；0.5 = 纵向压扁一半。\n"
            "需要勾上「自动缩放」才生效（不勾可以鼠标滚轮/拖动自由缩放）。"
        )
        row_view.addWidget(self.sp_zoom, 0, 1)

        row_view.addWidget(QtWidgets.QLabel("时间窗 (s):"), 1, 0)
        self.sp_win = QtWidgets.QDoubleSpinBox()
        self.sp_win.setRange(0.0, 10.0)
        self.sp_win.setSingleStep(0.1)
        self.sp_win.setDecimals(2)
        self.sp_win.setValue(0.0)
        self.sp_win.setMinimumWidth(88)
        self.sp_win.setToolTip(
            "横向显示最近多少秒，0 = 显示全部缓冲。\n"
            "窗口拉小时自动保持 1ms 原始分辨率（不会被抽点抽掉）。\n"
            "例如 0.20 就是只看最近 200ms，适合看 PWM/电流细节。"
        )
        row_view.addWidget(self.sp_win, 1, 1)

        self.btn_view_reset = QtWidgets.QPushButton("重置视图")
        self.btn_view_reset.setToolTip("纵向缩放回 1.0、时间窗回 0（显示全部）")
        self.btn_view_reset.clicked.connect(self.reset_view)
        row_view.addWidget(self.btn_view_reset, 2, 0, 1, 2)

        disp_lay = QtWidgets.QVBoxLayout()
        disp_lay.setSpacing(4)
        disp_lay.addLayout(row3)
        disp_lay.addLayout(row_view)
        pv.addWidget(self._group("显示与视图", disp_lay))

        # 自动缩放没勾时，纵向缩放没有意义，直接置灰
        self.chk_auto.toggled.connect(self.sp_zoom.setEnabled)
        self.sp_zoom.setEnabled(self.chk_auto.isChecked())

        # 左：控制面板（可滚动）  右：图像；连接栏横跨顶部
        panel.setMinimumWidth(430)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidget(panel)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)
        scroll.setMinimumWidth(440)
        scroll.setMaximumWidth(600)

        split = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        for pid, _text, _chans in PLOT_DEFS:
            split.addWidget(self.plots[pid])
        split.setChildrenCollapsible(False)

        # ---------- 频响页签（贝德图） ----------
        self.pw_mag = pg.PlotWidget(title="幅值 (dB)")
        self.pw_phase = pg.PlotWidget(title="相位 (°)")
        for pw in (self.pw_mag, self.pw_phase):
            pw.showGrid(x=True, y=True, alpha=0.3)
            pw.setLogMode(x=True, y=False)
            pw.setLabel("bottom", "频率", units="Hz")
        self.pw_mag.setLabel("left", "|H|", units="dB")
        self.pw_phase.setLabel("left", "相位", units="°")

        self.curve_mag = self.pw_mag.plot(pen=pg.mkPen((120, 220, 255), width=2),
                                          symbol="o", symbolSize=5)
        self.curve_phase = self.pw_phase.plot(pen=pg.mkPen((255, 200, 80), width=2),
                                              symbol="o", symbolSize=5)

        bode_widget = QtWidgets.QWidget()
        bl = QtWidgets.QVBoxLayout(bode_widget)
        bl.setContentsMargins(0, 0, 0, 0)
        bl.addWidget(self.pw_mag, 1)
        bl.addWidget(self.pw_phase, 1)

        self.tabs = QtWidgets.QTabWidget()
        self.tabs.addTab(split, "波形")
        self.tabs.addTab(bode_widget, "频响")

        # ---------- 扫频 / 参数辨识页签 ----------
        ident_widget = QtWidgets.QWidget()
        il = QtWidgets.QVBoxLayout(ident_widget)
        il.setContentsMargins(8, 8, 8, 8)
        il.setSpacing(8)

        sw = QtWidgets.QGroupBox("自动扫频")
        sl = QtWidgets.QGridLayout(sw)
        sl.setHorizontalSpacing(8)
        sl.setVerticalSpacing(6)

        self.cb_sweep = QtWidgets.QComboBox()
        self.cb_sweep.addItem("电流环扫频", 3)
        self.cb_sweep.addItem("转速环扫频", 4)
        sl.addWidget(QtWidgets.QLabel("对象:"), 0, 0)
        sl.addWidget(self.cb_sweep, 0, 1)

        self.sp_sweep_amp = QtWidgets.QDoubleSpinBox()
        self.sp_sweep_amp.setRange(0.05, 5000.0)
        self.sp_sweep_amp.setSingleStep(0.1)
        self.sp_sweep_amp.setDecimals(2)
        self.sp_sweep_amp.setValue(0.3)
        self.sp_sweep_amp.setToolTip("电流环扫频单位 A；转速环扫频单位 rpm")
        sl.addWidget(QtWidgets.QLabel("幅值:"), 1, 0)
        sl.addWidget(self.sp_sweep_amp, 1, 1)

        self.btn_sweep = QtWidgets.QPushButton("开始自动扫频")
        self.btn_sweep.clicked.connect(self.start_sweep)
        self.btn_sweep_stop = QtWidgets.QPushButton("停止")
        self.btn_sweep_stop.clicked.connect(self.stop_sweep)
        sl.addWidget(self.btn_sweep, 2, 0)
        sl.addWidget(self.btn_sweep_stop, 2, 1)

        self.lbl_sweep = QtWidgets.QLabel("扫频: 空闲")
        self.lbl_sweep.setStyleSheet("font-family:Consolas,monospace")
        sl.addWidget(self.lbl_sweep, 3, 0, 1, 2)
        il.addWidget(sw)

        ig = QtWidgets.QGroupBox("参数辨识")
        gl = QtWidgets.QGridLayout(ig)
        gl.setHorizontalSpacing(8)
        gl.setVerticalSpacing(6)

        self.btn_rs = QtWidgets.QPushButton("识别 Rs")
        self.btn_rs.clicked.connect(self.start_rs_ident)
        self.btn_l = QtWidgets.QPushButton("识别 Ld/Lq")
        self.btn_l.clicked.connect(self.start_l_ident)
        self.btn_jb = QtWidgets.QPushButton("识别 J/B")
        self.btn_jb.clicked.connect(self.start_jb_ident)
        gl.addWidget(self.btn_rs, 0, 0)
        gl.addWidget(self.btn_l, 0, 1)
        gl.addWidget(self.btn_jb, 0, 2)

        self.lbl_ident = QtWidgets.QLabel("Rs --   Ld --   Lq --   J --   B --")
        self.lbl_ident.setStyleSheet("font-family:Consolas,monospace; color:#d0d0d0;")
        gl.addWidget(self.lbl_ident, 1, 0, 1, 3)
        il.addWidget(ig)

        il.addStretch(1)
        self.tabs.addTab(ident_widget, "扫频 / 辨识")

        body = QtWidgets.QHBoxLayout()
        body.setContentsMargins(0, 0, 0, 0)
        body.setSpacing(6)
        body.addWidget(scroll)
        body.addWidget(self.tabs, 1)

        central = QtWidgets.QWidget()
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(4, 4, 4, 4)
        root.setSpacing(4)
        root.addWidget(self._group("连接", row1))
        root.addLayout(body, 1)
        self.setCentralWidget(central)

        # ================= 状态栏 =================
        self.lbl_status = QtWidgets.QLabel("未连接")
        self.lbl_stats = QtWidgets.QLabel("")
        self.statusBar().addWidget(self.lbl_status, 1)
        self.statusBar().addPermanentWidget(self.lbl_stats)

        self.refresh_ports()
        self.on_mode_changed(self.cb_mode.currentIndex())

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(40)

        self.timer_stats = QtCore.QTimer(self)
        self.timer_stats.timeout.connect(self.update_stats)
        self.timer_stats.start(500)

        # 命令 20Hz 重发
        self.timer_cmd = QtCore.QTimer(self)
        self.timer_cmd.timeout.connect(self.send_command)
        self.timer_cmd.start(CMD_PERIOD_MS)

        # 参数辨识状态机
        self.timer_ident = QtCore.QTimer(self)
        self.timer_ident.setSingleShot(True)
        self.timer_ident.timeout.connect(self._ident_tick)
        self._ident = None
        self._rs_value = float("nan")
        self._j_value = float("nan")
        self._b_value = float("nan")
        self._last_test_freq = 0.0
        self._test_type = 0
        self._sweep_loop = 0

        self.chk_sine.toggled.connect(self._on_sine_toggled)
        self.cb_sine.currentIndexChanged.connect(self._on_sine_toggled)

        self._apply_visibility()

    # ---------------- 小工具
    @staticmethod
    def _sync_slider(slider, value):
        if slider.value() != value:
            slider.blockSignals(True)
            slider.setValue(value)
            slider.blockSignals(False)

    @staticmethod
    def _gain_spin(prefix, lo, hi, step, decimals, value):
        sp = QtWidgets.QDoubleSpinBox()
        sp.setPrefix(prefix + " ")
        sp.setRange(lo, hi)
        sp.setSingleStep(step)
        sp.setDecimals(decimals)
        sp.setValue(value)
        sp.setMinimumWidth(105)
        return sp

    @staticmethod
    def _group(title, layout):
        box = QtWidgets.QGroupBox(title)
        lay = QtWidgets.QVBoxLayout(box)
        lay.setContentsMargins(10, 10, 10, 8)
        lay.setSpacing(4)
        lay.addLayout(layout)
        return box

    def _on_tune_toggled(self, on):
        if on:
            self.read_pi_from_fw()

    def _on_tune_sp_toggled(self, on):
        if on:
            self.read_pi_sp_from_fw()

    def read_pi_from_fw(self):
        """把固件回显的当前 Kp/Ki 填进输入框（先用它当前值做起点再调）"""
        kp_d, ki_d, kp_q, ki_q = self.store.pi
        if not all(math.isfinite(x) and x > 0.0 for x in (kp_d, ki_d, kp_q, ki_q)):
            return
        for sp, val in ((self.sp_kp_d, kp_d), (self.sp_ki_d, ki_d),
                        (self.sp_kp_q, kp_q), (self.sp_ki_q, ki_q)):
            sp.setValue(val)

    def on_mode_changed(self, idx):
        mode = self.cb_mode.itemData(idx)
        if mode is None:
            return

        # 切模式先把输出关掉：
        #   1) 新模式的默认目标可能让电机立刻大转矩跳一下（位置环默认 0°）
        #   2) 冲击电流会把限流电源 / 板载 USB 串口拉挂，
        #      现象就是"一改控制模式，串口就再也收不到数据"
        self.btn_enable.setChecked(False)

        cur_mode = (mode == MODE_CURRENT)
        openloop_mode = (mode == MODE_OPENLOOP)
        speed_mode = (mode == MODE_ENCODER)
        position_mode = (mode == MODE_POSITION)

        # 开环强拖同时用 iq 作为幅值、rpm 作为目标转速
        iq_enabled = cur_mode or openloop_mode
        rpm_enabled = speed_mode or openloop_mode
        pos_enabled = position_mode

        self.sp_iq.setEnabled(iq_enabled)
        self.sl_iq.setEnabled(iq_enabled)
        self.sp_rpm.setEnabled(rpm_enabled)
        self.sl_rpm.setEnabled(rpm_enabled)
        self.sp_pos.setEnabled(pos_enabled)
        self.sp_id.setEnabled(mode != MODE_OPENLOOP and mode != MODE_VIEW)

        if openloop_mode:
            self.lbl_iq.setText("强拖电压系数:")
            self.lbl_rpm.setText("强拖转速 (rpm):")
            self.sp_iq.setToolTip(
                "开环强拖没有电流环！这里的数值只决定电压幅值，不是电流：\n"
                "调制度 = |输入| × 0.2（0.1 → 2% 母线电压）\n"
                "实际电流 = 调制度 × Vbus/√3 ÷ 电机相电阻，会随电机不同\n"
                "想要电流恒定请用「电流闭环」"
            )
            self.sp_rpm.setToolTip("开环强拖目标转速，按加速度斜坡爬升；建议从静止启动、先给小幅值")
        elif position_mode:
            # 切到位置环时目标先取当前角度：先原地保持，不要猛地回 0°
            cur_deg = self.store.enc[0]
            if math.isfinite(cur_deg):
                self.sp_pos.setValue(cur_deg)

            self.lbl_iq.setText("电流目标 (A):")
            self.lbl_rpm.setText("转速目标 (rpm):")
            self.sp_iq.setToolTip("")
            self.sp_rpm.setToolTip("")
        else:
            self.lbl_iq.setText("电流目标 (A):")
            self.lbl_rpm.setText("转速目标 (rpm):")
            self.sp_iq.setToolTip("")
            self.sp_rpm.setToolTip("")

        self.btn_enable.setEnabled(mode != MODE_VIEW)

    # ---------------- 串口
    def refresh_ports(self):
        cur = self.cb_port.currentText()
        self.cb_port.clear()
        for p in list_ports.comports():
            self.cb_port.addItem(f"{p.device} - {p.description}", p.device)
        if cur:
            idx = self.cb_port.findText(cur)
            if idx >= 0:
                self.cb_port.setCurrentIndex(idx)

    def _start_reader(self, reader):
        reader.error.connect(self.on_reader_error)
        reader.start()
        self.reader = reader
        self._connected_at = time.time()

    def toggle_connect(self, checked):
        if checked:
            port = self.cb_port.currentData()
            if port is None:
                port = self.cb_port.currentText().split(" - ")[0].strip()
            if not port:
                QtWidgets.QMessageBox.warning(self, "提示", "没有可用串口，插好 USB 后点「刷新」")
                self.btn_connect.setChecked(False)
                return
            self.store.clear()
            self._start_reader(SerialReader(port, self.sp_baud.value(), self.store))
            self._port_text = f"已连接 {port} @ {self.sp_baud.value()} bps"
            self.lbl_status.setText(self._port_text)
            self.btn_connect.setText("断开")
        else:
            self._stop_reader()
            self.lbl_status.setText("未连接")

    def _stop_reader(self):
        if self.reader is not None:
            self.reader.stop()
            self.reader = None
        self.btn_connect.setText("连接")
        self.btn_connect.setChecked(False)
        self.btn_enable.setChecked(False)

    def start_demo(self):
        self.store.clear()
        self._start_reader(DemoReader(self.store))
        self.btn_connect.setChecked(True)
        self.btn_connect.setText("断开")
        self._port_text = "演示模式（模拟数据，没有真实串口）"
        self.lbl_status.setText(self._port_text)

    def on_reader_error(self, msg):
        self.lbl_status.setText(msg)
        self._stop_reader()

    def request_calibration(self):
        # 零偏校准必须让电机停住、输出关断，否则会把残余电流当成零点
        self.btn_enable.setChecked(False)
        self.cal_flag = True

    def request_angle_zero(self):
        self.angle_zero_flag = True

    def request_angle_calibration(self):
        # 标定由固件强制给 d 轴电流并开输出，先把用户输出关掉更安全
        self.btn_enable.setChecked(False)
        self.angle_cal_flag = True

    # ---------------- 参数辨识 / 扫频 ----------------
    def _set_mode(self, mode):
        idx = self.cb_mode.findData(mode)
        if idx >= 0:
            self.cb_mode.setCurrentIndex(idx)

    def _on_sine_toggled(self, _=None):
        if self.chk_sine.isChecked():
            self._test_type = int(self.cb_sine.currentData())
        elif self._test_type in (1, 2):
            self._test_type = 0

    def start_sweep(self):
        loop = int(self.cb_sweep.currentData())

        self.chk_sine.setChecked(False)
        self._set_mode(MODE_CURRENT if loop == 3 else MODE_ENCODER)
        self.btn_enable.setChecked(True)

        if loop == 3:
            self.sp_id.setValue(0.0)
            self.sp_iq.setValue(0.0)
        else:
            if abs(self.sp_rpm.value()) < 100.0:
                self.sp_rpm.setValue(300.0)
            if self.sp_sweep_amp.value() < 5.0:
                self.sp_sweep_amp.setValue(50.0)

        self._sweep_loop = loop
        self._test_type = 0                    # 先停一下
        self._pending_sweep = loop
        self.lbl_sweep.setText("扫频: 启动中…")
        QtCore.QTimer.singleShot(250, self._apply_pending_sweep)

    def _apply_pending_sweep(self):
        self._test_type = getattr(self, "_pending_sweep", 0)

    def stop_sweep(self):
        self._test_type = 0
        self._sweep_loop = 0
        self.lbl_sweep.setText("扫频: 已停止")

    def _start_l_test(self):
        self._test_type = 5

    def start_rs_ident(self):
        self._test_type = 0
        self._sweep_loop = 0
        self._set_mode(MODE_CURRENT)
        self.btn_enable.setChecked(True)
        self.sp_iq.setValue(0.0)
        self.sp_id.setValue(1.0)
        self._ident = {"kind": "rs", "step": 0, "v1": 0.0, "i1": 0.0}
        self.timer_ident.start(600)

    def start_l_ident(self):
        self._test_type = 0
        self._sweep_loop = 0
        self._set_mode(MODE_CURRENT)
        self.btn_enable.setChecked(True)
        self.chk_sine.setChecked(False)

        # 250ms 后再置 5，保证固件看到 0 → 5 的跳变
        QtCore.QTimer.singleShot(250, self._start_l_test)

    def start_jb_ident(self):
        self._test_type = 0
        self._sweep_loop = 0
        self._set_mode(MODE_CURRENT)
        self.btn_enable.setChecked(True)
        self.sp_id.setValue(0.0)

        # 用 iq 框里的值；如果是 0 就用 0.3A 起步
        iq = self.sp_iq.value()
        if abs(iq) < 0.05:
            iq = 0.3

        # 先停住再加速，避免上一次的余速混进初始斜率
        self.sp_iq.setValue(0.0)
        self._ident = {"kind": "jb", "step": 0, "iq": iq}
        self.timer_ident.start(800)

    def _ident_tick(self):
        ident = self._ident
        if ident is None:
            return

        st = self.store

        if ident["kind"] == "rs":
            with st.lock:
                id_avg = sum(list(st.chan["id"])[-50:]) / max(1, len(list(st.chan["id"])[-50:]))
                vd = st.vd

            if ident["step"] == 0:
                ident["v1"] = vd
                ident["i1"] = id_avg
                ident["step"] = 1
                self.sp_id.setValue(2.0)
                self.timer_ident.start(600)
            else:
                di = id_avg - ident["i1"]
                if abs(di) > 0.05:
                    self._rs_value = (vd - ident["v1"]) / di
                self.sp_id.setValue(0.0)
                self._ident = None
                self._update_ident_label()
            return

        if ident["kind"] == "jb":
            if ident["step"] == 0:
                # 先让电机停稳，再给 iq 加速
                ident["step"] = 1
                self.sp_iq.setValue(ident["iq"])
                self.timer_ident.start(1200)
                return

            with st.lock:
                spd = np.array(list(st.chan["speed"]), dtype=float)

            if len(spd) > 300:
                seg = spd[-1200:]
                n0 = min(100, len(seg))
                t0 = np.arange(n0) / SAMPLE_RATE_HZ
                slope = float(np.polyfit(t0, seg[:n0], 1)[0])      # rpm/s
                rpm_ss = float(np.mean(seg[-100:]))

                kt = 1.5 * 7 * 0.0024        # N·m/A
                iq = ident.get("iq", 0.8)

                # 加速度太小 / 没转起来就判无效
                if abs(slope) > 50.0:
                    self._j_value = kt * iq / (slope * (2.0 * math.pi / 60.0))
                else:
                    self._j_value = float("nan")

                if abs(rpm_ss) > 20.0:
                    self._b_value = kt * iq / (rpm_ss * (2.0 * math.pi / 60.0))
                else:
                    self._b_value = float("nan")

            self.sp_iq.setValue(0.0)
            self._ident = None
            self._update_ident_label()

    def _update_ident_label(self):
        ld = self.store.ld_x10 / 10.0
        lq = self.store.lq_x10 / 10.0

        def fmt(v, unit="", digits=4):
            return "---" if not math.isfinite(v) else f"{v:.{digits}f}{unit}"

        self.lbl_ident.setText(
            f"Rs {fmt(self._rs_value, 'Ω')}   "
            f"Ld {ld:.1f}µH   Lq {lq:.1f}µH   "
            f"J {fmt(self._j_value, '', 3)}   B {fmt(self._b_value, '', 4)}")

    def compute_bode(self):
        """扫频结束后，按 test_freq 分段做单频拟合，画伯德图"""
        with self.store.lock:
            f = np.array(list(self.store.chan["test_freq"]), dtype=float)
            if len(f) == 0:
                return

            loop = self._sweep_loop

            if loop == 3:
                x = np.array(list(self.store.chan["iq_t"]), dtype=float)
                y = np.array(list(self.store.chan["iq"]), dtype=float)
            else:
                x = np.array(list(self.store.chan["speed_t"]), dtype=float)
                y = np.array(list(self.store.chan["speed"]), dtype=float)

        n = len(f)
        freqs, mags, phases = [], [], []

        i = 0
        while i < n:
            if f[i] <= 0.0:
                i += 1
                continue

            fv = f[i]
            j = i
            while j < n and abs(f[j] - fv) < 1e-3:
                j += 1

            if j - i >= 50:
                k = min(j - i, max(50, int(3.0 * SAMPLE_RATE_HZ / max(fv, 0.1))))
                xs = x[j - k:j]
                ys = y[j - k:j]
                t = np.arange(k) / SAMPLE_RATE_HZ
                w = np.exp(-2j * np.pi * fv * t)
                X = np.sum(xs * w)
                Y = np.sum(ys * w)

                if abs(X) > 1e-9:
                    H = Y / X
                    freqs.append(float(fv))
                    mags.append(float(20.0 * np.log10(abs(H) + 1e-12)))
                    phases.append(float(np.degrees(np.angle(H))))

            i = j

        if freqs:
            order = np.argsort(freqs)
            fr = np.array(freqs)[order]
            self.curve_mag.setData(fr, np.array(mags)[order])
            self.curve_phase.setData(fr, np.array(phases)[order])
            self.tabs.setCurrentIndex(1)

    def read_pi_sp_from_fw(self):
        """把固件回显的转速/位置环参数填进输入框"""
        kp_s, ki_s, kp_p, kd_p = self.store.pi_sp
        for sp, val in ((self.sp_spd_kp, kp_s), (self.sp_spd_ki, ki_s),
                        (self.sp_pos_kp, kp_p), (self.sp_pos_kd, kd_p)):
            if math.isfinite(val):
                sp.setValue(val)

    def send_command(self):
        if self.reader is None:
            return

        mode = self.cb_mode.currentData()
        if mode is None:
            return

        enable = 1 if (self.btn_enable.isChecked() and mode != MODE_VIEW) else 0
        iq_ma = int(self.sp_iq.value() * 1000.0)
        rpm = int(self.sp_rpm.value())

        flags = 0
        if self.cal_flag:
            flags |= FLAG_CAL_CURRENT
            self.cal_flag = False
        if self.angle_zero_flag:
            flags |= FLAG_RESET_ANGLE
            self.angle_zero_flag = False
        if self.angle_cal_flag:
            flags |= FLAG_CAL_ANGLE
            self.angle_cal_flag = False

        # 电流环 PI 在线整定：不勾就发 0 且不带 bit3，固件会恢复上电默认参数
        if self.chk_tune.isChecked():
            flags |= FLAG_TUNE_PI
            kp_d = self.sp_kp_d.value()
            ki_d = self.sp_ki_d.value()
            kp_q = self.sp_kp_q.value()
            ki_q = self.sp_ki_q.value()
        else:
            kp_d = ki_d = kp_q = ki_q = 0.0

        # 转速环 PI / 位置环 PD 在线整定
        if self.chk_tune_sp.isChecked():
            flags |= FLAG_TUNE_SPEED_PI | FLAG_TUNE_POS_PI
            speed_kp = self.sp_spd_kp.value()
            speed_ki = self.sp_spd_ki.value()
            pos_kp = self.sp_pos_kp.value()
            pos_kd = self.sp_pos_kd.value()
        else:
            speed_kp = speed_ki = pos_kp = pos_kd = 0.0

        # 测试信号：正弦 / 扫频 / L 辨识 由 _test_type 统一决定
        test_type = int(self._test_type)
        test_amp = 0.0
        test_freq = 0.0

        if test_type in (1, 2):
            test_amp = self.sp_sine_amp.value()
            test_freq = self.sp_sine_freq.value()
        elif test_type in (3, 4):
            test_amp = self.sp_sweep_amp.value()
            test_freq = 0.0
        else:
            test_type = 0

        self.reader.send(build_cmd(enable, mode, iq_ma, rpm, flags,
                                   int(round(self.sp_pos.value() * 100.0)),
                                   kp_d, ki_d, kp_q, ki_q,
                                   int(round(self.sp_id.value() * 1000.0)),
                                   speed_kp, speed_ki, pos_kp, pos_kd,
                                   test_type, test_amp, test_freq))

    # ---------------- 显示
    def toggle_pause(self, checked):
        self.paused = checked
        self.btn_pause.setText("继续" if checked else "暂停")

    def reset_view(self):
        self.sp_zoom.setValue(1.0)
        self.sp_win.setValue(0.0)

    # ---------------- 显示控制 ----------------
    def visible_keys(self):
        keys = []
        for pid, _text, chans in PLOT_DEFS:
            if self.chk_plot[pid].isChecked():
                keys.extend(chans)
        return keys

    def _apply_refresh(self, hz):
        self.timer.start(max(10, int(1000 / max(1, hz))))

    def _set_plots(self, on, only=None):
        for pid, _text, _chans in PLOT_DEFS:
            cb = self.chk_plot[pid]
            cb.blockSignals(True)
            cb.setChecked(on if only is None else (pid in only))
            cb.blockSignals(False)
        self._apply_visibility()

    def _apply_visibility(self):
        for pid, _text, _chans in PLOT_DEFS:
            self.plots[pid].setVisible(self.chk_plot[pid].isChecked())

    @staticmethod
    def _moving_avg(y, win=SMOOTH_WINDOW):
        if win <= 1 or len(y) < win:
            return y
        c = np.cumsum(np.insert(y, 0, 0.0))
        out = (c[win:] - c[:-win]) / win
        pad = np.full(len(y) - len(out), out[0], dtype=np.float32)
        return np.concatenate([pad, out])

    def update_plots(self):
        if self.paused:
            return

        keys = self.visible_keys()
        if not keys:
            return

        # 时间窗：0 = 全部；否则只取最后 win 秒，保持 1ms 分辨率
        win = self.sp_win.value()
        tail = int(round(win * SAMPLE_RATE_HZ)) if win > 1e-6 else None

        snap = self.store.snapshot(keys, MAX_PLOT_POINTS, tail)
        if snap is None:
            return

        data, n, step = snap

        # 抽点后 x 轴也要跟着抽，否则时间轴对不上
        idx = np.arange(0, n, step, dtype=np.float32)
        t = (idx - (n - 1)) / SAMPLE_RATE_HZ

        smooth = self.chk_smooth.isChecked()

        for k in keys:
            y = data[k]
            if smooth and k in ("ia", "ib", "ic"):
                y = self._moving_avg(y)
            self.curves[k].setData(t, y, skipFiniteCheck=True)

        if n > 1:
            self.pw_cur.setXRange(float(t[0]), float(t[-1]), padding=0)

        # 纵向：按可见曲线自动取范围，再除以缩放倍数（1.0 = 铺满）
        if self.chk_auto.isChecked() and n > 1:
            zoom = max(0.05, self.sp_zoom.value())
            for pw in {self.plot_of[k] for k in keys}:
                lo = None
                hi = None
                for k, p in self.plot_of.items():
                    if p is not pw or k not in data:
                        continue
                    y = data[k]
                    if y is None or len(y) == 0:
                        continue
                    ymin = float(np.nanmin(y))
                    ymax = float(np.nanmax(y))
                    if not (math.isfinite(ymin) and math.isfinite(ymax)):
                        continue
                    lo = ymin if lo is None else min(lo, ymin)
                    hi = ymax if hi is None else max(hi, ymax)

                if lo is None:
                    continue

                mid = 0.5 * (lo + hi)
                half = max(0.5 * (hi - lo), 1e-3) / zoom
                pw.setYRange(mid - half, mid + half, padding=0)

    def update_stats(self):
        if self.reader is None:
            return

        frames, lost, status, raw, isr, enc, iq_now, spd, spd_t, v, elec_offset, pi, offs, iq_t, id_now, id_t, pi_sp = self.store.last_values()
        now = time.time()
        dt = now - self._last_time
        fps = (frames - self._last_frames) / dt if dt > 0 else 0
        self._last_frames = frames
        self._last_time = now

        mode_txt = self.cb_mode.currentText()
        fw_mode = (status >> 12) & 0x07
        fw_mode_txt = FW_MODE_TEXT.get(fw_mode, f"未知{fw_mode}")
        cmd_ok = bool(status & 0x8000)
        mode_display = mode_txt if fw_mode_txt == mode_txt else f"{mode_txt}/固件:{fw_mode_txt}"
        flags = []
        if status & 0x01:
            flags.append("已使能")
        if status & 0x02:
            flags.append("转速环")
        if status & 0x04:
            flags.append("编码器 OK")
        if status & 0x08:
            flags.append("电流零偏校准中")
        if status & 0x10:
            flags.append("ADC校准失败")
        if status & 0x20:
            flags.append("电角度标定中")
        if status & 0x40:
            flags.append("电角度标定完成")
        if status & 0x80:
            flags.append("电角度标定失败")
        if not cmd_ok:
            flags.append("命令丢失")

        if frames == 0 and (now - self._connected_at) > 2.0:
            if self.store.rx_bytes == 0:
                self.lbl_status.setText(
                    "已连接但一个字节都没收到：固件没跑起来 / 选错 COM / 波特率不是 4000000"
                )
            else:
                typ, ln = self.store.proto
                if typ is None:
                    self.lbl_status.setText(
                        f"收到 {self.store.rx_bytes} 字节，但找不到帧头 A5 5A："
                        "波特率不对，或者数据源不是这个固件"
                    )
                else:
                    self.lbl_status.setText(
                        f"收到 {self.store.rx_bytes} 字节，帧头对但 type={typ} len={ln}"
                        f"（上位机期望 type=1 len={PAYLOAD_LEN}）→ 固件和上位机版本不一致，重新烧录固件"
                    )

        # 没有母线电压时，电流必然是 0，直接提示，免得以为采样坏了
        if frames > 0:
            no_bus = (v == v) and (v < 5.0)
            if no_bus and self.cb_mode.currentData() != MODE_VIEW:
                self.lbl_status.setText(
                    "母线 0V（没接功率电）：改目标只会改变 SVPWM 占空比，电流恒为 0；接 12V 才能看到电流跟随"
                )
            elif self.lbl_status.text().startswith("母线 0V"):
                self.lbl_status.setText(self._port_text)

        # FOC 中断耗时：0.1us -> us；20kHz 周期 50us
        isr_avg = isr[0] / 10.0
        isr_max = isr[1] / 10.0
        load = isr_avg / 50.0 * 100.0

        stage = (status >> 8) & 0x0F
        stage_txt = STAGE_TEXT[stage] if stage < len(STAGE_TEXT) else f"阶段{stage}"

        spd_label = "设定" if (status & 0x02) else "转速"

        self.lbl_stats.setText(
            f"{mode_display} | {'/'.join(flags) if flags else '未使能'} | {stage_txt} | "
            f"raw {raw[0]}/{raw[1]}/{raw[2]} | off {offs[0]}/{offs[1]}/{offs[2]} | "
            f"iq {iq_now:5.2f}/{iq_t:5.2f}  id {id_now:5.2f}/{id_t:5.2f} | "
            f"{spd_label} {spd:5.0f}/{spd_t:5.0f}rpm | "
            f"角度 {enc[0]:8.1f}°({enc[0]/360.0:+.2f}圈)/{enc[1]:4.0f}rpm(坏帧{enc[2]}) | "
            f"offset {elec_offset:+6.2f}° | {v:5.1f}V | "
            f"PI {pi[0]:.2f}/{pi[1]:.0f}/{pi[2]:.2f}/{pi[3]:.0f} | 丢帧 {lost}"
        )

        # ISR 占用率放顶栏的独立标签，永远可见（状态栏太长会被截掉）
        self.lbl_isr.setText(
            f"ISR {isr_avg:5.1f}/{isr_max:5.1f}us  {load:3.0f}%  {fps:3.0f}fps"
        )

        # 扫频结束检测：test_freq 从 >0 回到 0
        tf = self.store.chan["test_freq"][-1] if self.store.chan["test_freq"] else 0.0
        if self._last_test_freq > 0.0 and tf <= 0.0:
            try:
                self.compute_bode()
            except Exception:
                pass
            self._test_type = 0
            self._sweep_loop = 0
            self.lbl_sweep.setText("扫频: 完成")
        elif self._test_type in (3, 4) and tf > 0.0:
            self.lbl_sweep.setText(f"扫频: {tf:.1f} Hz")
        self._last_test_freq = tf

        self._update_ident_label()
        if load >= 80.0:
            color = "#ff5555"
        elif load >= 50.0:
            color = "#ffcc66"
        else:
            color = "#88ff88"
        self.lbl_isr.setStyleSheet(f"color:{color}; font-family:Consolas,monospace")

    def closeEvent(self, event):            # noqa: N802
        self._stop_reader()
        super().closeEvent(event)


def self_test():
    """不接硬件时验证协议：解析 3 帧（含 1 帧坏校验），并检查命令组帧。"""
    def make(seq, bad=False):
        payload = struct.pack(UNPACK_FMT, 1.0, -2.0, 3.0, 0.5, 0.4, 0.3,
                              0.1, -0.2, 24000, 1234, 0x05, seq, 2048, 2050, 2046, 42, 98,
                              123.4, 56, 0, 0x123456, 12.34,
                              0.235, 1162.0, 0.397, 1162.0,
                              1993, 2052, 2107, 0.42, 0.0,
                              0.0682, 1.35, 80.0, 0.0, 500.0, 90.0, 50.0, 0.12, 0.0, 745, 425)
        body = bytes((TYPE_SCOPE, len(payload))) + payload
        chk = 0
        for b in body:
            chk ^= b
        if bad:
            chk ^= 0xFF
        return bytes((HEAD0, HEAD1)) + body + bytes((chk,))

    parser = FrameParser()
    frames = parser.feed(make(1) + b"\x00\x11" + make(2, bad=True) + make(3))
    assert len(frames) == 2, frames
    assert frames[0][8] == 24000 and frames[0][9] == 1234 and frames[0][11] == 1, frames[0]
    assert frames[0][12] == 2048 and frames[0][14] == 2046, frames[0]
    assert frames[0][15] == 42 and frames[0][16] == 98, frames[0]
    assert abs(frames[0][17] - 123.4) < 0.01 and frames[0][18] == 56, frames[0]
    assert frames[0][20] == 0x123456, frames[0]
    assert abs(frames[0][22] - 0.235) < 1e-5 and abs(frames[0][23] - 1162.0) < 1e-2, frames[0]
    assert abs(frames[0][24] - 0.397) < 1e-5 and abs(frames[0][25] - 1162.0) < 1e-2, frames[0]
    assert frames[0][26] == 1993 and frames[0][27] == 2052 and frames[0][28] == 2107, frames[0]
    assert abs(frames[0][29] - 0.42) < 1e-5, frames[0]
    assert abs(frames[0][30]) < 1e-6, frames[0]
    assert abs(frames[0][31] - 0.0682) < 1e-5 and abs(frames[0][32] - 1.35) < 1e-5, frames[0]
    assert abs(frames[0][35] - 500.0) < 1e-6, frames[0]
    assert abs(frames[0][36] - 90.0) < 1e-6, frames[0]
    assert abs(frames[0][33] - 80.0) < 1e-4 and abs(frames[0][34]) < 1e-6, frames[0]

    cmd = build_cmd(1, MODE_ENCODER, 1500, 300)
    assert len(cmd) == 4 + CMD_PAYLOAD + 1
    chk = 0
    for b in cmd[2:-1]:
        chk ^= b
    assert chk == cmd[-1]
    assert struct.unpack(CMD_FMT, cmd[4:-1])[:4] == (1, MODE_ENCODER, 1500, 300)

    openloop_cmd = build_cmd(1, MODE_OPENLOOP, 2000, 500)
    assert struct.unpack(CMD_FMT, openloop_cmd[4:-1])[:4] == (1, MODE_OPENLOOP, 2000, 500)

    tune_cmd = build_cmd(1, MODE_CURRENT, 0, 0, FLAG_TUNE_PI, 0, 0.3, 1200.0, 0.4, 1300.0)
    u = struct.unpack(CMD_FMT, tune_cmd[4:-1])
    assert u[4] == FLAG_TUNE_PI, u
    assert abs(u[6] - 0.3) < 1e-6 and abs(u[7] - 1200.0) < 1e-3, u
    assert abs(u[8] - 0.4) < 1e-6 and abs(u[9] - 1300.0) < 1e-3, u

    sp_cmd = build_cmd(1, MODE_ENCODER, 0, 100, FLAG_TUNE_SPEED_PI | FLAG_TUNE_POS_PI,
                       0, 0, 0, 0, 0, 0, 0.07, 1.5, 90.0, 0.0)
    s = struct.unpack(CMD_FMT, sp_cmd[4:-1])
    assert s[4] == (FLAG_TUNE_SPEED_PI | FLAG_TUNE_POS_PI), s
    assert abs(s[11] - 0.07) < 1e-6 and abs(s[12] - 1.5) < 1e-5, s
    assert abs(s[13] - 90.0) < 1e-4 and abs(s[14]) < 1e-6, s

    print("协议自检通过：", frames)


def main():
    ap = argparse.ArgumentParser(description="BeyondFOC 上位机")
    ap.add_argument("--demo", action="store_true", help="用模拟数据演示界面")
    ap.add_argument("--self-test", action="store_true", help="只跑协议自检")
    args = ap.parse_args()

    if args.self_test:
        self_test()
        return 0

    app = QtWidgets.QApplication(sys.argv)
    win = ScopeWindow()
    win.show()
    if args.demo:
        win.start_demo()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
