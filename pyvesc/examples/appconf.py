# --- appconf_test.py ---
import sys
import time
import pprint
import copy
import pyvesc

# --- 필요한 모듈/클래스/함수 import ---
try:
    from pyvesc.protocol.interface import encode_request
    from pyvesc.protocol.packet.codec import unframe
    from pyvesc.VESC.messages.getters import GetAppConfRequest  # APPCONF Getter
    from pyvesc.VESC.messages.setters import SetAppConf  # APPCONF Setter

    # !!!!! APPCONF 파서 import !!!!!
    from pyvesc.VESC.messages.parser import parse_app_conf_serialized

    # !!!!! APPCONF 인코딩 헬퍼 import !!!!!
    from pyvesc.VESC.messages.vesc_protocol_utils import (
        encode_set_appconf,
    )  # 경로 확인!

    # 헬퍼 함수 import
    from pyvesc.VESC.messages.helper import (
        connect_to_vesc,
        close_serial_port,
        clear_input_buffer,
        TIMEOUT,
    )  # TIMEOUT도 가져오기

except ImportError as e:
    print(f"오류: import 실패 ({e}). 경로 확인 필요.")
    sys.exit(1)

# --- 설정 ---
SERIAL_PORT = "/dev/cu.usbmodem3041"
BAUD_RATE = 115200
# TIMEOUT 은 helper 에서 가져옴

# --- 메인 로직 ---
if __name__ == "__main__":
    vesc_serial = None
    original_app_config = None

    print("=" * 50)
    print("VESC APPCONF GET/SET 테스트")
    print("=" * 50)
    print("!!! 경고: SET 기능은 VESC 설정을 변경합니다. 주의! !!!")

    try:
        vesc_serial = connect_to_vesc(SERIAL_PORT, BAUD_RATE, TIMEOUT)
        if not vesc_serial:
            sys.exit(1)

        # --- 단계 1: 현재 APPCONF 읽기 ---
        print("\n--- [단계 1] 현재 APPCONF 읽기 ---")
        print("  GET_APPCONF 요청...")
        clear_input_buffer(vesc_serial)
        request_get = encode_request(GetAppConfRequest)  # ID 17
        vesc_serial.write(request_get)
        time.sleep(TIMEOUT)
        # APPCONF 크기는 작을 수 있음, 1024 정도로도 충분할 수 있음
        response_get = vesc_serial.read(4096)

        if not response_get:
            print("  오류: GET_APPCONF 응답 없음.")
            sys.exit(1)

        print(f"  GET_APPCONF 응답 수신 ({len(response_get)} 바이트).")
        try:
            payload_get, consumed_get = unframe(response_get)
            if payload_get and payload_get[0] == GetAppConfRequest.id:  # ID 17 확인
                # !!!!! APPCONF 파서 호출 !!!!!
                original_app_config = parse_app_conf_serialized(payload_get[1:])
                if original_app_config and "APPCONF_SIGNATURE" in original_app_config:
                    print("  현재 APPCONF 파싱 성공.")
                    print("\n--- 파싱된 APPCONF (일부) ---")
                    pprint.pprint(
                        {
                            k: original_app_config[k]
                            for k in list(original_app_config)[:15]
                        }
                    )  # 앞 15개 출력
                    print("--------------------------")
                else:
                    print("  오류: APPCONF 파싱 실패 또는 Signature 없음.")
                    original_app_config = None
            else:
                print(f"  GET_APPCONF 응답 Unframing/ID 오류.")
                original_app_config = None
        except Exception as e:
            print(f"  GET_APPCONF 처리 오류: {e}")
            original_app_config = None

        # --- 단계 2: APPCONF 수정 및 쓰기 ---
        if original_app_config:
            run_set_test = input(
                "\nAPPCONF 쓰기 테스트를 진행하시겠습니까? (y/n): "
            ).lower()
            if run_set_test == "y":
                print("\n--- [단계 2] APPCONF 수정 및 쓰기 ---")
                config_to_write = copy.deepcopy(original_app_config)
                config_to_write.pop("crc", None)  # CRC는 없지만 안전하게

                # 수정할 필드와 값 선택 (예: UART Baudrate 변경)
                field_to_change = "app_uart_baudrate"
                current_baud = config_to_write.get(field_to_change, "N/A")
                new_baud = 9600 if current_baud != 9600 else 115200  # 토글 예시
                print(
                    f"  필드 '{field_to_change}' 값 변경 시도: {current_baud} -> {new_baud}"
                )
                config_to_write[field_to_change] = new_baud

                if "APPCONF_SIGNATURE" not in config_to_write:
                    print("!!! 오류: 설정에 Signature가 없습니다.")
                    sys.exit(1)

                try:
                    set_message = SetAppConf()
                    set_message.app_configuration = config_to_write  # 속성 이름 확인
                    if set_message.id != 16:
                        print("경고: SetAppConf ID 불일치")

                    # APPCONF 인코딩 헬퍼 사용
                    set_packet = encode_set_appconf(set_message)

                    print(f"  SET_APPCONF 요청 (ID {set_message.id}) 전송...")
                    clear_input_buffer(vesc_serial)
                    bytes_written = vesc_serial.write(set_packet)
                    print(f"  {bytes_written} 바이트 전송 완료. 처리 대기...")
                    time.sleep(1.0)
                    print("  SET_APPCONF 전송 완료.")

                    # --- 단계 3: 변경 확인 ---
                    print("\n--- [단계 3] 변경 사항 확인 ---")
                    # get_current_mcconf 대신 get_current_appconf 함수 구현 필요
                    # 여기서는 간단히 다시 GET 요청 보내고 파싱해서 값 확인
                    print("  확인용 GET_APPCONF 요청...")
                    clear_input_buffer(vesc_serial)
                    request_verify = encode_request(GetAppConfRequest)
                    vesc_serial.write(request_verify)
                    time.sleep(TIMEOUT)
                    response_verify = vesc_serial.read(4096)
                    verified_value = "확인 실패"
                    if response_verify:
                        payload_verify, consumed_verify = unframe(response_verify)
                        if payload_verify and payload_verify[0] == GetAppConfRequest.id:
                            verified_config = parse_app_conf_serialized(
                                payload_verify[1:]
                            )
                            if verified_config:
                                verified_value = verified_config.get(
                                    field_to_change, "Not Found"
                                )
                    print(f"  확인된 '{field_to_change}' 값: {verified_value}")
                    if verified_value == new_baud:
                        print("\n  >> 성공: 값이 성공적으로 변경되었습니다! <<")
                    else:
                        print("\n  >> 실패: 값이 변경되지 않았거나 예상과 다릅니다. <<")

                except Exception as e:
                    print(f"\n!!! APPCONF 쓰기 중 오류 발생: {e} !!!")
                    import traceback

                    traceback.print_exc()
            else:
                print("  APPCONF 쓰기 테스트를 건너<0xEB><0x9A><0x84>니다.")
        else:
            print("\n현재 APPCONF 설정을 읽지 못해 SET 테스트를 진행할 수 없습니다.")

    except KeyboardInterrupt:
        print("\n사용자에 의해 중단됨.")
    except Exception as e:
        print(f"\n!!! 스크립트 실행 중 오류 발생: {e} !!!")
        import traceback

        traceback.print_exc()
    finally:
        close_serial_port(vesc_serial)
        print("\n스크립트 종료.")
