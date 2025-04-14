# VESC 시리얼 통신 + l_current_max 읽기/쓰기 (COMM_GET_MCCONF, COMM_SET_MCCONF)

import serial
import struct
import time

# ----------------------------------------
# 설정
# ----------------------------------------
PORT = "/dev/cu.usbmodem3041"
BAUDRATE = 115200
TIMEOUT = 2.0


# ----------------------------------------
# CRC 계산 (VESC 표준 CRC16-CCITT)
# ----------------------------------------
def crc16(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc


# ----------------------------------------
# 패킷 빌더
# ----------------------------------------
def build_packet(comm_id, payload=b""):
    length = len(payload) + 1  # +1 for comm_id
    packet = bytearray()
    packet.append(2)  # Start
    packet += struct.pack(">H", length)
    packet.append(comm_id)
    packet += payload
    crc = crc16(packet[3:])  # from comm_id to end of payload
    packet += struct.pack(">H", crc)
    packet.append(3)  # End
    return packet


# ----------------------------------------
# 시리얼 연결
# ----------------------------------------
def connect_serial():
    try:
        ser = serial.Serial(PORT, baudrate=BAUDRATE, timeout=TIMEOUT)
        time.sleep(2)
        ser.reset_input_buffer()
        print(f"[✅] 연결됨: {PORT}")
        return ser
    except Exception as e:
        print(f"[❌] 연결 실패: {e}")
        return None


# ----------------------------------------
# l_current_max 읽기
# ----------------------------------------
def read_l_current_max(ser):
    COMM_GET_MCCONF = 4
    packet = build_packet(COMM_GET_MCCONF)
    ser.write(packet)
    time.sleep(0.5)
    resp = ser.read(512)

    if not resp or len(resp) < 10:
        print("[❌] 응답 없음 또는 너무 짧음")
        return

    # 응답에서 payload 부분 추출
    payload = resp[6:-3]  # ID + SUBID 1바이트 후 추정
    l_current_max_bytes = payload[0:4]
    l_current_max = struct.unpack(">f", l_current_max_bytes)[0]
    print(f"[📥] 현재 l_current_max: {l_current_max:.2f} A")


# ----------------------------------------
# l_current_max 쓰기
# ----------------------------------------
def write_l_current_max(ser, new_value):
    COMM_SET_MCCONF = 5
    l_current_max_bytes = struct.pack(">f", new_value)
    payload = l_current_max_bytes + bytes(300)  # 기본값 0으로 채움 (총 길이 맞추기)
    packet = build_packet(COMM_SET_MCCONF, payload)
    ser.write(packet)
    print(f"[📤] l_current_max 설정값 {new_value} A 전송 완료")


# ----------------------------------------
# 실행
# ----------------------------------------
if __name__ == "__main__":
    ser = connect_serial()
    if ser:
        read_l_current_max(ser)
        write_l_current_max(ser, 55.0)
        time.sleep(1)
        read_l_current_max(ser)
        ser.close()
