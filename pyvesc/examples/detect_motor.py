import sys
import os
import serial
import time
import struct
import pprint
import copy  # for deep copy
import pyvesc

# --- pyvesc 및 필요한 컴포넌트 import ---
try:
    from pyvesc.protocol.packet.codec import frame, unframe
    from pyvesc.protocol.interface import encode, encode_request
    from pyvesc.VESC.messages.getters import GetMcConfRequest
    from pyvesc.VESC.messages.setters import (
        DetectMotorRL,
        # SetMcConf 는 아래 apply 함수 내에서 사용
        SetMcConf,
    )
    from pyvesc.VESC.messages.parser import (
        parse_mc_conf_serialized,
        pack_mc_conf_serialized,
    )

except ImportError as e:
    print(f"오류: 필요한 pyvesc 모듈/클래스 import 실패 ({e}).")
    sys.exit(1)
except AttributeError as e:
    print(f"오류: 클래스 속성 관련 문제 ({e}).")
    sys.exit(1)

# --- 설정 ---
SERIAL_PORT = "/dev/cu.usbmodem3041"
BAUD_RATE = 115200
TIMEOUT = 1.0
DETECTION_TIMEOUT = 15.0  # R/L 감지는 비교적 짧음

# --- VESC 설정 값 (펌웨어 확인 필요!) ---
MOTOR_TYPE_FOC = 2  # !!! 실제 VESC 펌웨어의 MOTOR_TYPE_FOC 값 확인 !!!
TEMP_FOC_F_ZV = 10000.0  # C 코드에서 사용된 임시 f_zv 값


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


# --- GET MCCONF 함수 ---
def get_current_mcconf(ser):
    print("  GET_MCCONF 요청 (설정 읽기)...")
    clear_input_buffer(ser)
    request = encode_request(GetMcConfRequest)
    try:
        ser.write(request)
        time.sleep(TIMEOUT)
        response = ser.read(4096)
        if response:
            payload, consumed = unframe(response)
            if payload and payload[0] == GetMcConfRequest.id:
                print("  GET 응답 Unframing 성공.")
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


# --- SET MCCONF 함수 (헬퍼) ---
def set_current_mcconf(ser, config_dict, description="설정"):
    """주어진 설정 딕셔너리를 VESC에 씁니다."""
    if not config_dict or "MCCONF_SIGNATURE" not in config_dict:
        print(
            f"!!! 오류: {description} 쓰기 실패 - 유효하지 않은 설정 데이터 (Signature 없음?)"
        )
        return False

    print(f"  SET_MCCONF 요청 ({description} 쓰기)...")
    # Packer가 Signature를 사용하고 pop한다고 가정하므로 깊은 복사본 전달
    config_to_write = copy.deepcopy(config_dict)
    config_to_write.pop("crc", None)  # CRC는 제거

    try:
        # SetMcConf 인스턴스 생성 및 데이터 할당
        set_message = SetMcConf()
        set_message.mc_configuration = config_to_write

        # 패킷 인코딩 (내부적으로 packer -> frame 호출)
        # encode_set_mcconf 헬퍼 대신 직접 처리
        print("    Packer 함수 호출 (Signature 자동 사용)...")
        data_payload = pack_mc_conf_serialized(set_message.mc_configuration)
        if not data_payload:
            raise ValueError("Packer가 빈 데이터를 반환했습니다.")
        print(f"    데이터 페이로드 생성됨 (크기: {len(data_payload)})")

        command_id_byte = bytes([set_message.id])  # ID: 13 가정
        full_payload = command_id_byte + data_payload
        print(f"    frame() 입력 페이로드 생성됨 (크기: {len(full_payload)})")

        set_packet = frame(full_payload)
        print(f"    최종 패킷 생성됨 (크기: {len(set_packet)})")

        # 전송
        clear_input_buffer(ser)
        bytes_written = ser.write(set_packet)
        print(f"    {bytes_written} 바이트 전송 완료.")
        print(f"    VESC 처리 대기 (1.0초)...")
        time.sleep(1.0)  # VESC가 플래시에 쓸 시간
        print(f"  {description} 쓰기 완료.")
        return True

    except KeyError as e:
        print(f"!!! 패킹 오류 (KeyError): '{e}'. 설정 데이터 확인 필요.")
        return False
    except Exception as e:
        print(f"!!! {description} 쓰기 중 오류 발생: {e}")
        import traceback

        traceback.print_exc()
        return False


# --- 메인 실행 로직 ---
if __name__ == "__main__":
    vesc_serial = None
    original_config = None  # 원래 설정을 저장할 변수
    measured_r = None
    measured_l = None
    measured_ld_lq_diff = None

    print("=" * 60)
    print("    VESC R/L 감지 테스트 (임시 설정 적용 및 복원)")
    print("=" * 60)
    print("\n!!! [경고] 안전 절차를 반드시 준수하세요 !!!")
    # ... (안전 경고는 이전과 동일) ...
    print("=" * 60)

    confirm = input("R/L 감지를 실행하시겠습니까? (y/n): ")
    if confirm.lower() != "y":
        print("실행 취소됨.")
        sys.exit(0)

    try:
        vesc_serial = connect_to_vesc(SERIAL_PORT, BAUD_RATE, TIMEOUT)
        if not vesc_serial:
            sys.exit(1)

        # --- 단계 1: 현재 설정 읽기 (백업) ---
        print("\n--- [단계 1] 현재 설정 읽기 (원본 백업) ---")
        original_config = get_current_mcconf(vesc_serial)
        if not original_config:
            print("!!! 오류: 원본 설정을 읽을 수 없어 안전하게 종료합니다.")
            sys.exit(1)
        print("  원본 설정 읽기 성공.")
        # print(f"  (원본 motor_type: {original_config.get('motor_type')}, foc_f_zv: {original_config.get('foc_f_zv')})")

        # --- 단계 2: 임시 설정 생성 및 쓰기 ---
        print("\n--- [단계 2] 감지용 임시 설정 생성 및 적용 ---")
        temp_config = copy.deepcopy(original_config)  # 깊은 복사
        print(
            f"  임시 설정 변경: motor_type -> {MOTOR_TYPE_FOC}, foc_f_zv -> {TEMP_FOC_F_ZV}"
        )
        temp_config["motor_type"] = MOTOR_TYPE_FOC
        temp_config["foc_f_zv"] = TEMP_FOC_F_ZV

        if not set_current_mcconf(vesc_serial, temp_config, "임시 설정"):
            print("!!! 오류: 임시 설정 적용 실패. 원본 설정 복원을 시도합니다.")
            # 복원 시도 후 종료 (finally 에서 처리)
            raise RuntimeError("임시 설정 적용 실패")  # finally 로 점프

        print("  임시 설정 적용 완료.")

        # --- 단계 3: R/L 감지 명령 보내기 ---
        print("\n--- [단계 3] R/L 감지 명령 전송 ---")
        detection_message = DetectMotorRL()  # 페이로드 없음
        packet_to_send = encode_request(detection_message.__class__)
        clear_input_buffer(vesc_serial)
        print(f"  R/L 감지 명령 (ID: {detection_message.id}) 전송...")
        bytes_written = vesc_serial.write(packet_to_send)
        print(f"  {bytes_written} 바이트 전송 완료.")

        # --- 단계 4: R/L 결과 응답 대기 및 파싱 ---
        print(f"\n--- [단계 4] R/L 결과 응답 대기 (최대 {DETECTION_TIMEOUT}초) ---")
        detection_response = b""
        start_time = time.time()
        response_received_in_time = False
        while time.time() - start_time < DETECTION_TIMEOUT:
            if vesc_serial.in_waiting > 0:
                time.sleep(0.2)  # 데이터 수신 시간 확보
                detection_response += vesc_serial.read(vesc_serial.in_waiting)
                response_received_in_time = True
                break
            time.sleep(0.1)

        if not response_received_in_time:
            print("  !!! 오류: R/L 감지 응답 시간 초과 !!!")
            # 복원 필요 (finally 에서 처리)
        else:
            print(
                f"  R/L 응답 수신 ({len(detection_response)} 바이트). Unframing 및 파싱..."
            )
            try:
                response_payload, consumed = unframe(detection_response)
                if response_payload and response_payload[0] == DetectMotorRL.id:
                    print(f"  R/L 응답 Unframing 성공 (ID: {DetectMotorRL.id}).")
                    response_data = response_payload[1:]
                    print(f"  응답 데이터 (Hex): {response_data.hex(' ')}")

                    # 응답 형식: >Biii (ID + R + L + Ld/Lq) - 스케일링된 int32 가정
                    expected_len = 12
                    if len(response_data) >= expected_len:
                        r_raw, l_raw, ld_lq_diff_raw = struct.unpack_from(
                            ">iii", response_data, 0
                        )
                        measured_r = float(r_raw) / 1000000.0  # scale 1e6
                        measured_l = float(l_raw) / 1000.0  # scale 1e3
                        measured_ld_lq_diff = (
                            float(ld_lq_diff_raw) / 1000.0
                        )  # scale 1e3
                        print("\n--- R/L 측정 결과 ---")
                        print(
                            f"  Resistance (R)      : {measured_r:.6f} Ohm (Raw: {r_raw})"
                        )
                        print(
                            f"  Inductance (L)      : {measured_l:.6f} H (Raw: {l_raw})"
                        )
                        print(
                            f"  Ld/Lq Difference    : {measured_ld_lq_diff:.6f} H (Raw: {ld_lq_diff_raw})"
                        )
                    else:
                        print(
                            f"  오류: R/L 응답 데이터 길이 부족 (Expected >= {expected_len}, Got {len(response_data)})"
                        )
                elif response_payload:
                    print(f"  오류: 예상치 못한 응답 ID (Got: {response_payload[0]})")
                else:
                    print("  오류: R/L 응답 Unframing 실패.")
            except struct.error as se:
                print(f"  오류: R/L 응답 파싱 실패 (struct.error: {se}).")
            except Exception as e:
                print(f"  R/L 응답 처리 중 오류: {e}")

    except KeyboardInterrupt:
        print("\n사용자에 의해 중단됨.")
    except Exception as e:
        print(f"\n!!! 스크립트 실행 중 오류 발생: {e} !!!")
        import traceback

        traceback.print_exc()
    finally:
        # --- 단계 5: 원본 설정 복원 (매우 중요!) ---
        print("\n--- [단계 5] 원본 설정 복원 시도 ---")
        if vesc_serial and vesc_serial.is_open and original_config:
            if set_current_mcconf(vesc_serial, original_config, "원본 설정"):
                print("  원본 설정 복원 완료.")
            else:
                print(
                    "!!! 중요 경고: 원본 설정 복원에 실패했습니다! VESC Tool로 설정을 확인/복구해야 할 수 있습니다."
                )
        elif not original_config:
            print("  오류: 복원할 원본 설정이 없습니다.")
        elif not (vesc_serial and vesc_serial.is_open):
            print("  오류: VESC 연결이 끊어져 원본 설정을 복원할 수 없습니다.")

        close_serial_port(vesc_serial)
        print("\n스크립트 종료.")
