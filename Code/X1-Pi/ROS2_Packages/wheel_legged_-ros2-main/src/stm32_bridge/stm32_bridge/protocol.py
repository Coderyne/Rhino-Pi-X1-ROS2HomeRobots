"""STM32 USB CDC 通讯协议: CRC16, 帧打包/解包, 遥测解析, 指令打包"""

import struct

# ── 帧格式常量 ──────────────────────────────────────────────
FRAME_HEADER_0 = 0xAA
FRAME_HEADER_1 = 0x55
TYPE_TELEMETRY = 0x01        # STM32 → ROS2
TYPE_COMMAND   = 0x02        # ROS2 → STM32

# ── 遥测帧 Payload 结构 ─────────────────────────────────────
TLM_FORMAT = '<23f4B'        # 23 个 float + 4 个 uint8, little-endian
TLM_SIZE   = struct.calcsize(TLM_FORMAT)  # 96 bytes

# ── 指令帧 Payload 结构 ─────────────────────────────────────
CMD_FORMAT = '<6fB'          # 6 个 float + 1 个 uint8
CMD_SIZE   = struct.calcsize(CMD_FORMAT)  # 25 bytes

# ── cmd_flags 位定义 ────────────────────────────────────────
CMD_ENABLE  = 0x01           # 使能底盘控制
CMD_JUMP    = 0x02           # 触发跳跃
CMD_ESTOP   = 0x04           # 紧急停止
CMD_RECOVER = 0x08           # 倒地自起

# ── 遥测字段名列表 (与 TLM_FORMAT 顺序一致) ──────────────────
TLM_FIELDS = [
    'timestamp', 'roll', 'pitch', 'yaw',
    'gyro_x', 'gyro_y', 'gyro_z',
    'accel_x', 'accel_y', 'accel_z',
    'vel_n', 'pos_n',
    'theta_L', 'L0_L', 'wheel_T_L', 'Tp_L', 'd_theta_L',
    'theta_R', 'L0_R', 'wheel_T_R', 'Tp_R', 'd_theta_R',
    'battery_voltage',
    'start_flag', 'jump_flag', 'contact_L', 'contact_R',
]


# ═══════════════════════════════════════════════════════════════
#  CRC16-CCITT
# ═══════════════════════════════════════════════════════════════

def crc16_ccitt(data: bytes) -> int:
    """CRC16-CCITT, poly=0x1021, init=0xFFFF"""
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc


# ═══════════════════════════════════════════════════════════════
#  帧打包 / 解包
# ═══════════════════════════════════════════════════════════════

def pack_frame(frame_type: int, payload: bytes) -> bytes:
    """
    打包完整数据帧
    帧格式: 0xAA 0x55 | Type(1B) | Len(1B) | Payload(Len B) | CRC16(2B LSB)
    CRC 覆盖: Type + Len + Payload
    """
    header = struct.pack('BBBB', FRAME_HEADER_0, FRAME_HEADER_1,
                         frame_type, len(payload))
    crc = crc16_ccitt(header[2:] + payload)
    return header + payload + struct.pack('<H', crc)


def unpack_frame(frame: bytes) -> tuple[int, bytes] | None:
    """
    解包数据帧, 校验 CRC 通过则返回 (type, payload), 否则返回 None
    frame 从 Type 字段开始 (不含帧头 0xAA 0x55), 必须等于:
        Type(1) + Len(1) + Payload(Len) + CRC16(2)
    """
    if len(frame) < 4:
        return None

    frame_type = frame[0]
    payload_len = frame[1]
    expected_len = 2 + payload_len + 2  # Type + Len + Payload + CRC

    if len(frame) != expected_len:
        return None

    payload = frame[2:2 + payload_len]
    crc_received = struct.unpack('<H', frame[2 + payload_len:expected_len])[0]

    # CRC 计算范围: Type + Len + Payload
    crc_calc = crc16_ccitt(frame[:2 + payload_len])
    if crc_calc != crc_received:
        return None

    return frame_type, payload


def verify_frame(frame: bytes) -> bool:
    """校验帧 CRC 是否通过 (帧包含 0xAA 0x55 头)"""
    if len(frame) < 6 or frame[0:2] != b'\xAA\x55':
        return False
    return unpack_frame(frame[2:]) is not None


# ═══════════════════════════════════════════════════════════════
#  逐字节帧解析器 (状态机) — CDC 拆包安全
# ═══════════════════════════════════════════════════════════════

class CDCState:
    WAIT_H0 = 0
    WAIT_H1 = 1
    WAIT_TYPE = 2
    WAIT_LEN = 3
    WAIT_DATA = 4


class FrameParser:
    """逐字节喂入的帧解析状态机, 输入 int(byte) 返回完整帧 payload 或 None"""

    def __init__(self):
        self._buf = bytearray()
        self._needed = 0       # 当前帧还需多少字节
        self._frame_type = 0   # 帧类型
        self._payload_len = 0  # payload 长度
        self._state = CDCState.WAIT_H0

    def feed(self, byte: int) -> tuple[int, bytes] | None:
        """
        喂入一个字节, 收到完整有效帧时返回 (type, payload), 否则返回 None
        """
        if self._state == CDCState.WAIT_H0:
            if byte == FRAME_HEADER_0:
                self._buf.append(byte)
                self._state = CDCState.WAIT_H1
            return None

        elif self._state == CDCState.WAIT_H1:
            if byte == FRAME_HEADER_1:
                self._buf.append(byte)
                self._state = CDCState.WAIT_TYPE
            elif byte == FRAME_HEADER_0:
                self._buf = bytearray([FRAME_HEADER_0])
            else:
                self._buf.clear()
                self._state = CDCState.WAIT_H0
            return None

        elif self._state == CDCState.WAIT_TYPE:
            self._buf.append(byte)
            self._frame_type = byte
            self._state = CDCState.WAIT_LEN
            return None

        elif self._state == CDCState.WAIT_LEN:
            self._buf.append(byte)
            self._payload_len = byte
            # Type(1) + Len(1) + Payload(Len) + CRC(2) = 4 + payload_len
            self._needed = 4 + self._payload_len - 2  # 已经收了 2 bytes
            self._state = CDCState.WAIT_DATA
            return None

        elif self._state == CDCState.WAIT_DATA:
            self._buf.append(byte)
            self._needed -= 1
            if self._needed == 0:
                # 完整帧已收集
                frame = bytes(self._buf)
                self._buf.clear()
                self._state = CDCState.WAIT_H0
                result = unpack_frame(frame[2:])  # 跳过 0xAA 0x55
                if result is not None:
                    return result
            return None

        return None

    def reset(self):
        """重置状态机"""
        self._buf.clear()
        self._state = CDCState.WAIT_H0
        self._needed = 0


# ═══════════════════════════════════════════════════════════════
#  遥测解析
# ═══════════════════════════════════════════════════════════════

def parse_telemetry(payload: bytes) -> dict:
    """解析遥测帧 payload, 返回字段字典"""
    if len(payload) != TLM_SIZE:
        return None
    fields = struct.unpack(TLM_FORMAT, payload)
    return dict(zip(TLM_FIELDS, fields))


# ═══════════════════════════════════════════════════════════════
#  指令打包
# ═══════════════════════════════════════════════════════════════

def pack_command(timestamp: float, v_set: float = 0.0,
                 yaw_rate_set: float = 0.0, roll_set: float = 0.0,
                 leg_set: float = 0.14, pitch_set: float = 0.0,
                 flags: int = 0) -> bytes:
    """
    打包控制指令帧
    返回完整帧 (含 0xAA 0x55 头 + CRC)
    """
    payload = struct.pack(CMD_FORMAT, timestamp, v_set, yaw_rate_set,
                          roll_set, leg_set, pitch_set, flags)
    return pack_frame(TYPE_COMMAND, payload)
