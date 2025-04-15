import sys
import os
import serial
import time
import struct
import pprint
import math  # for isnan, isinf (packer 함수 내부에서 사용 가정)
import pyvesc  # 사용자 정의 코드가 포함된 pyvesc 경로 설정 필요

# !!!!! pyvesc 의 패킹/프레이밍 함수 import !!!!!
try:
    from pyvesc.protocol.packet.codec import frame, unframe
    from pyvesc.protocol.interface import encode_request
except ImportError as e:
    print(f"오류: pyvesc.protocol 모듈 import 실패 ({e}). 라이브러리 확인 필요.")
    sys.exit(1)

# --- 사용자 정의 모듈/클래스/함수 가져오기 ---
try:
    from pyvesc.VESC.messages.getters import GetMcConfRequest

    # !!!!! setters.py 에서 SetMcConf 클래스를 import !!!!!
    from pyvesc.VESC.messages.setters import SetMcConf

    # !!!!! parser.py (또는 정의된 곳)에서 parser 와 packer 함수를 import !!!!!
    from pyvesc.VESC.messages.parser import (
        parse_mc_conf_serialized,
        pack_mc_conf_serialized,
    )
except ImportError as e:
    print(f"오류: 필요한 pyvesc 메시지 모듈/클래스/함수 import 실패.")
    print(f"    ({e})")
    sys.exit(1)
except AttributeError as e:
    print(f"오류: 클래스 속성 관련 문제.")
    print(f"    ({e}) - SetMcConf 클래스에 'fields = []' 정의 확인")
    sys.exit(1)

# --- 설정 ---
SERIAL_PORT = "/dev/cu.usbmodem3041"  # 자신의 VESC 포트로 변경!
BAUD_RATE = 115200
TIMEOUT = 1.0  # 응답 대기 시간 (초)

# --- 변경 테스트용 설정 ---
FIELD_TO_CHANGE = "l_current_max"
NEW_VALUE = 40.0


# --- Helper 함수 정의 (connect, close, clear) ---
def connect_to_vesc(port, baudrate, timeout):
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
    if ser and ser.is_open:
        try:
            ser.close()
            print("시리얼 포트 닫힘.")
        except Exception as e:
            print(f"시리얼 포트 닫기 오류: {e}")


def clear_input_buffer(ser, wait_time=0.1):
    if ser and ser.is_open:
        ser.reset_input_buffer()
        time.sleep(wait_time)
        if ser.in_waiting > 0:
            try:
                ser.read(ser.in_waiting)
            except Exception as e:
                print(f"  버퍼 비우기 중 오류: {e}")


# --- SetMcConf 패킷 인코딩 헬퍼 함수 (pyvesc의 frame 사용) ---
def encode_set_mcconf(message: SetMcConf):
    if not isinstance(message, SetMcConf):
        raise TypeError(f"message는 SetMcConf의 인스턴스여야 합니다.")
    # !!!!! 이제 message 객체에 mc_configuration 속성이 있는지 확인 !!!!!
    if not hasattr(message, "mc_configuration"):
        raise AttributeError(
            "encode_set_mcconf 호출 전 message 객체에 'mc_configuration' 속성을 할당해야 합니다."
        )

    try:
        print(
            "  Packer 함수(pack_mc_conf_serialized) 호출하여 데이터 페이로드 생성 중..."
        )
        # message.mc_configuration 에는 Signature가 포함되어 있어야 함
        data_payload = pack_mc_conf_serialized(message.mc_configuration)
        print(
            f"  생성된 데이터 페이로드(Signature포함) 크기: {len(data_payload)} 바이트"
        )

        command_id_byte = bytes([message.id])  # message.id 가 13이어야 함
        full_payload = command_id_byte + data_payload
        print(
            f"  frame() 입력 페이로드 (ID: {message.id} + Sig + Data) 크기: {len(full_payload)} 바이트"
        )

        print(f"  pyvesc.protocol.codec.frame 함수 호출하여 패킷 생성...")
        final_packet = frame(full_payload)
        print(f"  최종 패킷 생성 완료. 총 크기: {len(final_packet)} 바이트")

        return final_packet
    except KeyError as e:
        print(f"!!! 패킹 오류 (KeyError): '{e}'. 입력 딕셔너리 확인 필요.")
        raise
    except Exception as e:
        print(f"!!! SetMcConf 패킷 인코딩 중 오류: {e}")
        import traceback

        traceback.print_exc()
        raise


# --- 메인 실행 로직 ---
if __name__ == "__main__":
    vesc_serial = None
    parsed_motor_config = None

    print("=" * 50)
    try:
        print(f"VESC COMM_SET_MCCONF 테스트 (ID: {SetMcConf.id}, Signature 자동 감지)")
    except AttributeError:
        print(
            "VESC COMM_SET_MCCONF 테스트 (ID: 알 수 없음 - SetMcConf.id 확인 필요, Signature 자동 감지)"
        )
    print("=" * 50)
    print(f"포트: {SERIAL_PORT}, 속도: {BAUD_RATE}")
    print(f"변경 시도: '{FIELD_TO_CHANGE}' = {NEW_VALUE}")
    print("\n경고: VESC 설정을 변경합니다. 매우 주의하세요!")

    try:
        vesc_serial = connect_to_vesc(SERIAL_PORT, BAUD_RATE, TIMEOUT)
        if not vesc_serial:
            sys.exit(1)

        # --- 단계 1: 현재 MCCONF 읽기 ---
        print("\n--- [단계 1] 현재 설정 읽기 (GET_MCCONF) ---")
        clear_input_buffer(vesc_serial)
        get_request_packet = encode_request(GetMcConfRequest)  # ID: 14 가정
        print(f"  GET 요청 (ID {GetMcConfRequest.id}) 전송...")
        vesc_serial.write(get_request_packet)
        time.sleep(TIMEOUT)
        response_buffer = vesc_serial.read(4096)

        if not response_buffer:
            print("  오류: GET 응답 없음.")
            sys.exit(1)

        print(
            f"  GET 응답 수신 ({len(response_buffer)} 바이트). Unframing 및 파싱 시도..."
        )
        get_success = False
        try:
            unframed_payload, consumed = unframe(response_buffer)
            if unframed_payload:
                print(f"  Unframing 성공 (소비 {consumed} 바이트).")
                response_id = unframed_payload[0]
                if response_id == GetMcConfRequest.id:
                    actual_data_payload = unframed_payload[1:]
                    try:
                        parsed_motor_config = parse_mc_conf_serialized(
                            actual_data_payload
                        )
                        if (
                            parsed_motor_config
                            and "MCCONF_SIGNATURE" in parsed_motor_config
                        ):
                            print("  성공: 현재 모터 설정 파싱 완료.")
                            print(
                                f"    감지된 Signature: 0x{parsed_motor_config['MCCONF_SIGNATURE']:X}"
                            )
                            print(
                                f"    현재 '{FIELD_TO_CHANGE}' 값: {parsed_motor_config.get(FIELD_TO_CHANGE, 'Not Found')}"
                            )
                            get_success = True
                        else:
                            print("  오류: 파싱 실패 또는 Signature 없음.")
                    except Exception as e:
                        print(f"  파싱 중 오류: {e}")
                else:
                    print(f"  오류: Unframing 후 ID 불일치 (Got {response_id})")
            else:
                print(f"  오류: Unframing 실패 (소비 {consumed} 바이트).")
        except Exception as e:
            print(f"  Unframing/파싱 중 오류: {e}")

        # --- 단계 2: 설정 수정 및 쓰기 (GET 성공 시) ---
        if get_success and parsed_motor_config:
            print(
                f"\n--- [단계 2] 설정 수정 및 쓰기 (SET_MCCONF ID: {SetMcConf.id}) ---"
            )
            config_to_write = parsed_motor_config.copy()
            config_to_write.pop("crc", None)  # CRC는 제거 (Signature는 유지)

            if "MCCONF_SIGNATURE" not in config_to_write:
                print("!!! 오류: 복사된 설정에 Signature가 없습니다.")
                sys.exit(1)

            original_value = parsed_motor_config.get(FIELD_TO_CHANGE, "N/A")
            print(
                f"  필드 '{FIELD_TO_CHANGE}' 값 변경: {original_value} -> {NEW_VALUE}"
            )
            config_to_write[FIELD_TO_CHANGE] = NEW_VALUE

            try:
                # !!!!! 1. 빈 SetMcConf 인스턴스 생성 !!!!!
                set_message = SetMcConf()

                # !!!!! 2. 생성된 인스턴스에 설정 딕셔너리 할당 !!!!!
                set_message.mc_configuration = config_to_write

                # ID 일치 확인 (안전 장치)
                if set_message.id != 13:
                    print(
                        f"!!! 경고: 생성된 SetMcConf 메시지 ID({set_message.id})가 예상 값(13)과 다릅니다!"
                    )

                # 패킷 인코딩
                set_packet = encode_set_mcconf(set_message)

                # 전송
                print(f"  SET 요청 (ID {set_message.id}) 전송...")
                clear_input_buffer(vesc_serial)
                bytes_written = vesc_serial.write(set_packet)
                print(f"  {bytes_written} 바이트 전송 완료.")
                print(f"  VESC 처리 대기 (1.0초)...")
                time.sleep(1.0)

                # --- 단계 3: 변경 사항 확인 ---
                print("\n--- [단계 3] 변경 사항 확인 (다시 GET 요청) ---")
                # ... (이하 확인 로직은 이전과 동일) ...
                clear_input_buffer(vesc_serial)
                get_request_packet_verify = encode_request(GetMcConfRequest)
                print(f"  확인용 GET 요청 (ID {GetMcConfRequest.id}) 전송...")
                vesc_serial.write(get_request_packet_verify)
                time.sleep(TIMEOUT)
                response_buffer_verify = vesc_serial.read(4096)

                if not response_buffer_verify:
                    print("  오류: 확인 응답 없음.")
                else:
                    print(
                        f"  확인 응답 수신 ({len(response_buffer_verify)} 바이트). Unframing 및 파싱..."
                    )
                    verify_success = False
                    verified_value = "Error"
                    try:
                        unframed_payload_verify, consumed_verify = unframe(
                            response_buffer_verify
                        )
                        if unframed_payload_verify:
                            verify_response_id = unframed_payload_verify[0]
                            if verify_response_id == GetMcConfRequest.id:
                                actual_data_payload_verify = unframed_payload_verify[1:]
                                try:
                                    verified_config = parse_mc_conf_serialized(
                                        actual_data_payload_verify
                                    )
                                    if verified_config:
                                        print("  확인 파싱 성공.")
                                        verified_value = verified_config.get(
                                            FIELD_TO_CHANGE, "Not Found"
                                        )
                                        print(
                                            f"  확인된 '{FIELD_TO_CHANGE}' 값: {verified_value}"
                                        )
                                        verify_success = True
                                    else:
                                        print("  오류: 확인 파싱 실패.")
                                except Exception as e:
                                    print(f"  확인 파싱 중 오류: {e}")
                            else:
                                print(
                                    f"  오류: 확인 응답 ID 불일치 (Got {verify_response_id})"
                                )
                        else:
                            print(
                                f"  오류: 확인 Unframing 실패 (소비 {consumed_verify})."
                            )
                    except Exception as e:
                        print(f"  확인 Unframing/파싱 중 오류: {e}")

                    # 최종 결과 비교
                    if verify_success:
                        tolerance = 0.01
                        match = False
                        if isinstance(verified_value, float) and isinstance(
                            NEW_VALUE, float
                        ):
                            if abs(verified_value - NEW_VALUE) < tolerance:
                                match = True
                        elif verified_value == NEW_VALUE:
                            match = True

                        if match:
                            print("\n  ****************************************")
                        else:
                            print("\n  ########################################")
                        print(f"  * 결과: {'성공!' if match else '실패!'}")
                        print(f"  * 기대값: {NEW_VALUE}")
                        print(f"  * 확인값: {verified_value}")
                        if match:
                            print("  ****************************************")
                        else:
                            print("  ########################################")
                    else:
                        print("  오류: 최종 값 확인 실패 (파싱 오류 등)")

            except AttributeError as e:
                print(
                    f"!!! 오류: set_message 객체에 mc_configuration 속성이 없습니다. ({e})"
                )
                import traceback

                traceback.print_exc()
            except Exception as e:
                print(f"\n!!! 설정 쓰기/확인 중 오류 발생: {e} !!!")
                import traceback

                traceback.print_exc()

        else:
            print(
                "\n현재 모터 설정을 성공적으로 읽지 못하여 SET 명령을 진행할 수 없습니다."
            )

    except KeyboardInterrupt:
        print("\n사용자에 의해 중단됨.")
    except Exception as e:
        print(f"\n!!! 스크립트 실행 중 오류 발생: {e} !!!")
        import traceback

        traceback.print_exc()
    finally:
        close_serial_port(vesc_serial)
        print("\n스크립트 종료.")
