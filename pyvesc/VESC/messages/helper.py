# --- helper.py ---
import serial
import time
import sys  # 필요할 수 있음
import struct  # 필요할 수 있음 (get_current_mcconf 에서)

# !!!!! get_current_mcconf 가 사용하는 것들 import !!!!!
# 경로는 실제 프로젝트 구조에 맞게 조정 필요
try:
    from pyvesc.protocol.interface import encode_request
    from pyvesc.protocol.packet.codec import unframe
    from pyvesc.VESC.messages.getters import GetMcConfRequest
    from pyvesc.VESC.messages.parser import parse_mc_conf_serialized
except ImportError as e:
    print(f"오류(helper.py): 필요한 pyvesc 컴포넌트 import 실패 ({e}). 경로 확인 필요.")
    # 필요한 경우 여기서 프로그램을 종료하거나, 호출하는 쪽에서 처리하도록 함
    raise

# !!!!! TIMEOUT 상수 정의 (get_current_mcconf 에서 사용) !!!!!
# 또는 이 함수가 timeout 값을 인자로 받도록 수정할 수도 있음
TIMEOUT = 1.0


# --- 함수 정의 붙여넣기 ---
def connect_to_vesc(port, baudrate, timeout):
    # ... (함수 코드) ...
    print(f"포트 '{port}' ({baudrate} bps) 연결 시도...")
    try:
        ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        time.sleep(2)
        ser.reset_input_buffer()
        time.sleep(0.1)
        if ser.in_waiting > 0:
            ser.read(ser.in_waiting)
        if ser.is_open:
            print("성공: VESC 연결됨.")
            return ser
        else:
            print("오류: 포트 열기 실패.")
            return None
    except serial.SerialException as e:
        print(f"시리얼 연결 오류: {e}")
        return None
    except Exception as e:
        print(f"예상치 못한 연결 오류: {e}")
        return None


def close_serial_port(ser):
    # ... (함수 코드) ...
    if ser and ser.is_open:
        try:
            ser.close()
            print("시리얼 포트 닫힘.")
        except Exception as e:
            print(f"시리얼 포트 닫기 오류: {e}")


def clear_input_buffer(ser, wait_time=0.1):
    # ... (함수 코드) ...
    if ser and ser.is_open:
        ser.reset_input_buffer()
        time.sleep(wait_time)
        if ser.in_waiting > 0:
            try:
                ser.read(ser.in_waiting)
            except Exception as e:
                print(f"  버퍼 비우기 중 오류: {e}")


def get_current_mcconf(ser):
    # ... (함수 코드) ...
    print("  GET_MCCONF 요청 (설정 확인용)...")
    clear_input_buffer(ser)
    request = encode_request(GetMcConfRequest)
    try:
        ser.write(request)
        time.sleep(TIMEOUT)  # 위에 정의된 TIMEOUT 사용
        response = ser.read(4096)
        if response:
            payload, consumed = unframe(response)
            if payload and payload[0] == GetMcConfRequest.id:
                parsed_conf = parse_mc_conf_serialized(payload[1:])
                if parsed_conf and "MCCONF_SIGNATURE" in parsed_conf:
                    return parsed_conf
                else:
                    print("  오류: 파싱 실패 또는 Signature 없음.")
                    return None
            else:
                print(
                    f"  GET 응답 Unframing/ID 오류 (ID: {payload[0] if payload else 'N/A'})."
                )
                return None
        else:
            print("  GET 응답 없음.")
            return None
    except Exception as e:
        print(f"  GET_MCCONF 처리 중 오류: {e}")
        return None
