import sys
import os
import serial
import time
import struct
import pprint
import copy
import pyvesc

# --- pyvesc 및 필요한 컴포넌트 import ---
try:
    # frame/unframe 은 packet.codec 에 있음
    from pyvesc.protocol.packet.codec import *  # unframe 필요

    # encode_request 는 interface 에 있음
    from pyvesc.protocol.interface import *  # GET 요청에 필요

    # 메시지 클래스
    from pyvesc.VESC.messages.getters import *
    from pyvesc.VESC.messages.setters import *

    # 파서 (결과 확인용)
    from pyvesc.VESC.messages.parser import *

    # 패커 (유틸리티 함수 내부에서 사용됨)
    # from pyvesc.VESC.messages.parser import pack_mc_conf_serialized

    # !!!!! 새로 만든 유틸리티 파일에서 인코딩 함수 import !!!!!
    # vesc_protocol_utils.py 파일의 위치에 따라 경로 조정 필요
    from pyvesc.VESC.messages.vesc_protocol_utils import *

    from pyvesc.VESC.messages.helper import *


except ImportError as e:
    print(f"오류: 필요한 모듈/클래스/함수 import 실패 ({e}).")
    sys.exit(1)
except AttributeError as e:
    print(f"오류: 클래스 속성 문제 ({e}).")
    sys.exit(1)

# --- 설정 ---
SERIAL_PORT = "/dev/cu.usbmodem3041"
BAUD_RATE = 115200
TIMEOUT = 1.0

# --- 변경 테스트용 설정 ---
FIELD_TO_CHANGE = "l_current_max"
NEW_VALUE = 40.0  # 예시 값


# --- 메인 실행 로직 ---
if __name__ == "__main__":
    vesc_serial = None
    parsed_motor_config = None

    print("=" * 50)
    print("VESC COMM_SET_MCCONF 테스트 (설정 변경 및 쓰기)")
    print("=" * 50)
    print(f"포트: {SERIAL_PORT}, 속도: {BAUD_RATE}")
    print(f"변경 시도: '{FIELD_TO_CHANGE}' = {NEW_VALUE}")

    try:
        vesc_serial = connect_to_vesc(SERIAL_PORT, BAUD_RATE, TIMEOUT)
        if not vesc_serial:
            sys.exit(1)

        # --- 단계 1: 현재 설정 읽기 ---
        print("\n--- [단계 1] 현재 설정 읽기 ---")
        print("  GET_MCCONF 요청...")
        clear_input_buffer(vesc_serial)
        request_get = encode_request(GetMcConfRequest)
        vesc_serial.write(request_get)
        time.sleep(TIMEOUT)
        response_get = vesc_serial.read(4096)

        if not response_get:
            print("  오류: GET 응답 없음.")
            sys.exit(1)

        print(f"  GET 응답 수신 ({len(response_get)} 바이트).")
        try:
            payload_get, consumed_get = unframe(response_get)
            if payload_get and payload_get[0] == GetMcConfRequest.id:
                parsed_motor_config = parse_mc_conf_serialized(payload_get[1:])
                if parsed_motor_config and "MCCONF_SIGNATURE" in parsed_motor_config:
                    print("  현재 설정 파싱 성공.")
                    print(
                        f"    (현재 '{FIELD_TO_CHANGE}' 값: {parsed_motor_config.get(FIELD_TO_CHANGE, 'N/A')})"
                    )
                else:
                    raise ValueError("파싱 실패 또는 Signature 없음")
            else:
                raise ValueError(
                    f"GET 응답 Unframing/ID 오류 (ID: {payload_get[0] if payload_get else 'N/A'})"
                )
        except Exception as e:
            print(f"  GET 응답 처리 오류: {e}")
            sys.exit(1)

        # --- 단계 2: 설정 수정 및 쓰기 ---
        print(f"\n--- [단계 2] 설정 수정 및 쓰기 (SET_MCCONF ID: {SetMcConf.id}) ---")
        config_to_write = copy.deepcopy(parsed_motor_config)
        config_to_write.pop("crc", None)  # CRC 제거

        original_value = parsed_motor_config.get(FIELD_TO_CHANGE, "N/A")
        print(f"  필드 '{FIELD_TO_CHANGE}' 값 변경: {original_value} -> {NEW_VALUE}")
        config_to_write[FIELD_TO_CHANGE] = NEW_VALUE

        if "MCCONF_SIGNATURE" not in config_to_write:
            print("!!! 오류: 설정에 Signature가 없습니다.")
            sys.exit(1)

        try:
            set_message = SetMcConf()
            set_message.mc_configuration = config_to_write
            if set_message.id != 13:
                print(f"경고: SetMcConf ID 불일치({set_message.id})")

            # !!!!! import된 인코딩 헬퍼 사용 !!!!!
            set_packet = encode_set_mcconf(set_message)

            # 전송
            print(f"  SET 요청 (ID {set_message.id}) 전송...")
            clear_input_buffer(vesc_serial)
            bytes_written = vesc_serial.write(set_packet)
            print(f"  {bytes_written} 바이트 전송 완료.")
            print(f"  VESC 처리 대기 (1.0초)...")
            time.sleep(1.0)

            # --- 단계 3: 변경 사항 확인 ---
            print("\n--- [단계 3] 변경 사항 확인 ---")
            verified_config = get_current_mcconf(vesc_serial)  # 확인용 GET 요청
            if verified_config:
                verified_value = verified_config.get(FIELD_TO_CHANGE, "Not Found")
                print(f"  확인된 '{FIELD_TO_CHANGE}' 값: {verified_value}")
                # 값 비교
                tolerance = 0.01
                match = False
                if isinstance(verified_value, float) and isinstance(NEW_VALUE, float):
                    if abs(verified_value - NEW_VALUE) < tolerance:
                        match = True
                elif verified_value == NEW_VALUE:
                    match = True
                if match:
                    print("\n  >> 성공: 값이 성공적으로 변경되었습니다! <<")
                else:
                    print("\n  >> 실패: 값이 변경되지 않았거나 예상과 다릅니다. <<")
            else:
                print("  !!! 오류: 변경 후 설정을 읽을 수 없어 확인 불가 !!!")

        except Exception as e:
            print(f"\n!!! 설정 쓰기/확인 중 오류 발생: {e} !!!")
            import traceback

            traceback.print_exc()

    except KeyboardInterrupt:
        print("\n사용자에 의해 중단됨.")
    except Exception as e:
        print(f"\n!!! 스크립트 실행 중 오류 발생: {e} !!!")
        import traceback

        traceback.print_exc()
    finally:
        close_serial_port(vesc_serial)
        print("\n스크립트 종료.")
