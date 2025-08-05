import socket
import struct
import time
import math

# ==== Cấu hình IP Robot và cổng TCP ====
ROBOT_IP = "192.168.1.165"
ROBOT_PORT = 502
SOCK_TIMEOUT = 10.0  # tăng lên 10s cho ổn định


# ==== Helpers endian ====
def to_be_u16(value: int) -> bytes:
    return struct.pack(">H", int(value))


def recv_exact(sock: socket.socket, n: int) -> bytes:
    """Nhận chính xác n byte (tránh thiếu bytes gây lỗi)."""
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("Socket closed while receiving")
        buf += chunk
    return buf


# ==== Tạo khung truyền Lite6 ====
def create_lite6_packet(tid: int, register: int, params: bytes = b"") -> bytes:
    """
    Header BE:
      [TID u16][Proto u16=0x0002][Length u16]
    Body:
      [Register u8] + Params
    Length = 1 + len(Params)
    """
    proto = 0x0002
    length = 1 + len(params)
    return (
        to_be_u16(tid) +
        to_be_u16(proto) +
        to_be_u16(length) +
        bytes([register]) +
        params
    )


# ==== Gửi lệnh & nhận phản hồi ====
def send_command(packet: bytes, debug: bool = True) -> bytes:
    if debug:
        print("📤 TX:", packet.hex())
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(SOCK_TIMEOUT)
        sock.connect((ROBOT_IP, ROBOT_PORT))
        sock.sendall(packet)

        header = recv_exact(sock, 6)                       # 6 bytes
        tid, proto, length = struct.unpack(">HHH", header)
        body = recv_exact(sock, length) if length else b"" # length bytes

    rx = header + body
    if debug:
        print("📥 RX:", rx.hex())
    return rx


# ==== Giải mã phản hồi chung + riêng 0x17 ====
def parse_response(data: bytes) -> str:
    if len(data) < 6:
        return "Phản hồi không hợp lệ (thiếu header)"
    _, _, length = struct.unpack(">HHH", data[:6])
    body = data[6:6+length]
    if len(body) < 1:
        return "Phản hồi không hợp lệ (thiếu body)"

    reg = body[0]

    # Đặc tả riêng cho Register 0x17 (Joint motion P2P):
    # Body: [Reg u8][State u8][Parameter u16 BE]
    if reg == 0x17:
        if len(body) < 4:
            return f"Reg 0x17: body quá ngắn ({len(body)} bytes)"
        state = body[1]
        param = struct.unpack(">H", body[2:4])[0]
        state_text = "OK" if state == 0x00 else f"ERR(0x{state:02X})"
        return f"Reg:0x17, State:0x{state:02X} ({state_text}), Param:0x{param:04X}"

    # Mặc định kiểu cũ: [Reg u8][Status u8][...]
    status = body[1] if len(body) >= 2 else 0
    msgs = []
    if (status >> 6) & 1: msgs.append("Error(Bit6)")
    if (status >> 5) & 1: msgs.append("Warning(Bit5)")
    if (status >> 4) & 1: msgs.append("CannotMove(Bit4)")
    text = "OK" if not msgs else ", ".join(msgs)
    return f"Reg:0x{reg:02X}, Status:0x{status:02X} ({text})"


# ==== Lệnh chuẩn (Clear / Warning / Enable all joints) ====
def cmd_clear_error(tid: int = 1) -> bytes:
    return create_lite6_packet(tid, register=0x10)

def cmd_clear_warning(tid: int = 2) -> bytes:
    return create_lite6_packet(tid, register=0x11)

def cmd_enable_all_joints(tid: int = 3) -> bytes:
    # 0x0B: Param1=0x08 (all joints), Param2=0x01 (enable)
    return create_lite6_packet(tid, register=0x0B, params=b"\x08\x01")


# ==== Gripper (Register 0x7F, HostID/Addr theo manual) ====
def cmd_gripper(channel: int = 0, open_on: bool = True, tid: int = 10) -> bytes:
    HOST_ID = 0x09
    ADDR    = 0x0A15

    if channel not in (0, 1):
        raise ValueError("channel phải là 0 hoặc 1")
    if channel == 0:
        value = 257.0 if open_on else 256.0
    else:
        value = 514.0 if open_on else 512.0

    # Params = [HostID u8][Addr u16 BE][Value fp32 LE]
    params = bytes([HOST_ID]) + to_be_u16(ADDR) + struct.pack("<f", float(value))
    return create_lite6_packet(tid, register=0x7F, params=params)


def cmd_joint_motion_p2p_deg(
    j1_deg, j2_deg, j3_deg, j4_deg, j5_deg, j6_deg,
    speed_deg_s=20.0,
    acc_deg_s2=500.0,
    motion_time_s=0.0,
    tid=11,
    auto_send=True
):
    joints_deg = [j1_deg, j2_deg, j3_deg, j4_deg, j5_deg, j6_deg]
    if len(joints_deg) != 6:
        raise ValueError("Phải truyền đủ 6 góc độ (j1 đến j6)")

    j_rad = [math.radians(float(j)) for j in joints_deg]
    speed_rad_s = math.radians(speed_deg_s)
    acc_rad_s2 = math.radians(acc_deg_s2)

    # Đóng gói đúng 10 số float32 (LE)
    values = j_rad + [speed_rad_s, acc_rad_s2, float(motion_time_s)]
    assert len(values) == 10, "Thiếu giá trị để đóng gói"

    params = b''.join(struct.pack("<f", v) for v in values)
    assert len(params) == 40, "Params phải 40 bytes (10 * fp32)"

    packet = create_lite6_packet(tid, register=0x17, params=params)
    if not auto_send:
        return packet
    return parse_response(send_command(packet))



# ==== Demo ====
def demo():
    # Khuyến nghị: thử clear/enable trước.
    print(parse_response(send_command(cmd_clear_error(1))))
    print(parse_response(send_command(cmd_clear_warning(2))))
    print(parse_response(send_command(cmd_enable_all_joints(3))))

    # Joint motion theo độ: J1=60°, còn lại 0°, speed=20°/s, acc=500°/s², time=0
    print(cmd_joint_motion_p2p_deg(
        j1_deg=60, j2_deg=0, j3_deg=0, j4_deg=0, j5_deg=0, j6_deg=0,
        speed_deg_s=20, acc_deg_s2=500, motion_time_s=0.0,
        tid=11, auto_send=True
    ))

    # Ví dụ gripper (tuỳ robot có gắn hay không)
    # print(parse_response(send_command(cmd_gripper(channel=0, open_on=True, tid=21))))
    # time.sleep(1)
    # print(parse_response(send_command(cmd_gripper(channel=0, open_on=False, tid=22))))


if __name__ == "__main__":
    demo()
