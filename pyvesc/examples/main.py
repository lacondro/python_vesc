import sys
import os
import serial
import time
import pyvesc  # pyvesc 자체를 import 하기 전에 경로 설정 필요
import pprint
import struct  # parse 함수에서 사용될 수 있으므로 import (안전하게)

from pyvesc.VESC.messages.getters import (
    GetMcConfRequest,
)  # getters.py 에 정의했다고 가정
from pyvesc.VESC.messages.parser import parse_mc_conf_serialized
from pyvesc.protocol.base import VESCMessage


# --- 설정 ---
SERIAL_PORT = "/dev/cu.usbmodem3041"
BAUD_RATE = 115200
TIMEOUT = 1.0


# --- Helper 함수 정의 (누락된 부분 추가) ---
def connect_to_vesc(port, baudrate, timeout):
    print(f"포트 '{port}'에 {baudrate} 속도로 연결 시도 중...")
    try:
        ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        time.sleep(2)
        if ser.is_open:
            print(f"성공: 포트 '{port}'에 연결되었습니다.")
            ser.reset_input_buffer()
            time.sleep(0.1)
            initial_bytes = ser.in_waiting
            if initial_bytes > 0:
                print(f"  연결 후 초기 버퍼 데이터 {initial_bytes} 바이트 제거 중...")
                ser.read(initial_bytes)
            return ser
        else:
            return None
    except serial.SerialException as e:
        print(f"시리얼 연결 오류: {e}")
        return None
    except Exception as e:
        print(f"예상치 못한 오류 발생: {e}")
        return None


def close_serial_port(ser):
    if ser and ser.is_open:
        try:
            ser.close()
            print("시리얼 포트가 닫혔습니다.")
        except Exception as e:
            print(f"시리얼 포트 닫기 오류: {e}")


def clear_input_buffer(ser, wait_time=0.05):
    ser.reset_input_buffer()
    time.sleep(wait_time)
    bytes_to_read = ser.in_waiting
    if bytes_to_read > 0:
        try:
            ser.read(bytes_to_read)
        except Exception as e:
            print(f"  버퍼 비우기 중 오류: {e}")


# -------------------------------------------


# --- 메인 실행 부분 ---
if __name__ == "__main__":
    vesc_serial = None
    try:
        # *** 이제 함수가 정의되었으므로 오류 없이 호출 가능 ***
        vesc_serial = connect_to_vesc(SERIAL_PORT, BAUD_RATE, TIMEOUT)

        if vesc_serial:
            print("\n--- COMM_GET_MCCONF 요청 및 최종 파싱 시도 (모듈화) ---")
            print("요청 전 입력 버퍼 정리...")
            clear_input_buffer(vesc_serial, wait_time=0.1)

            # 요청 ID 14 사용 (getters.py에서 가져옴)
            # GetMcConfRequest를 GetCustomMcConfRequest로 임포트했음
            request_packet = pyvesc.encode_request(GetMcConfRequest)
            print("COMM_GET_MCCONF (ID 14) 요청 전송 중...")

            vesc_serial.write(request_packet)
            print("요청 전송 완료.")

            response_wait_time = 1.0
            print(f"{response_wait_time}초 동안 응답 대기...")
            time.sleep(response_wait_time)

            read_size = 4096
            print(f"최대 {read_size} 바이트 응답 읽기 시도...")
            mcconf_buffer = vesc_serial.read(read_size)

            if mcconf_buffer:
                print(f"\n수신 성공: {len(mcconf_buffer)} 바이트의 원시 데이터 수신.")
                # print(f"수신된 데이터 (앞 100바이트): {mcconf_buffer[:100]}...")

                header_size = 4
                if len(mcconf_buffer) > header_size:
                    response_id = mcconf_buffer[3]
                    print(f"  응답 패킷 ID: {response_id} (0x{response_id:02x})")
                    payload = mcconf_buffer[header_size:]

                    # 분리된 파싱 함수 호출 (vesc_parser.py에서 가져옴)
                    parsed_motor_config = parse_mc_conf_serialized(payload)

                    if (
                        parsed_motor_config
                        and "MCCONF_SIGNATURE" in parsed_motor_config
                    ):
                        print("\n--- 최종 파싱된 모터 구성 결과 ---")
                        pprint.pprint(parsed_motor_config)
                        print("---------------------------------")

                        # 값 비교
                        print("\n--- VESC Tool 값과 비교 ---")
                        vesc_tool_values = {
                            "l_current_max": 48.94,
                            "comm_mode": 0,
                            "foc_cc_decoupling": 0,
                            "foc_current_filter_const": 0.1,
                            "foc_current_ki": 33.40,
                            "foc_current_kp": 0.0532,
                        }
                        all_ok = True
                        for key, expected_value in vesc_tool_values.items():
                            status = "Not Found"
                            parsed_value_str = "N/A"
                            if key in parsed_motor_config:
                                parsed_value = parsed_motor_config[key]
                                parsed_value_str = (
                                    f"{parsed_value:<15.4f}"
                                    if isinstance(parsed_value, float)
                                    else f"{parsed_value:<15}"
                                )
                                if isinstance(parsed_value, float):
                                    if abs(parsed_value - expected_value) < 0.01:
                                        status = "OK"
                                    else:
                                        status = "MISMATCH"
                                        all_ok = False
                                elif parsed_value == expected_value:
                                    status = "OK"
                                else:
                                    status = "MISMATCH"
                                    all_ok = False
                            else:
                                all_ok = False
                            print(
                                f"Field: {key:<30} | Parsed: {parsed_value_str} | VESC Tool: {expected_value:<15} | Status: {status}"
                            )
                        print("-----------------------------")
                        if all_ok:
                            print(
                                "값 비교 결과: 모든 주요 값이 VESC Tool과 일치합니다!"
                            )
                        else:
                            print(
                                "값 비교 결과: 일부 값이 VESC Tool과 일치하지 않습니다."
                            )
                    else:
                        print(
                            "\n오류: 모터 구성 데이터 파싱에 실패했거나 유효하지 않습니다."
                        )
                else:
                    print("\n오류: 수신된 데이터가 너무 짧아 파싱할 수 없습니다.")
            else:
                print("\n오류: 응답 없음...")
        else:
            print("\nVESC 연결 실패.")
    except KeyboardInterrupt:
        print("\n사용자에 의해 프로그램 중단됨.")
    finally:
        # *** 이제 함수가 정의되었으므로 오류 없이 호출 가능 ***
        close_serial_port(vesc_serial)
        print("프로그램 종료.")
