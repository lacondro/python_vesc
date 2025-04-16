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
    from pyvesc.protocol.packet.codec import frame, unframe
    from pyvesc.protocol.interface import encode_request  # GET 요청에 필요

    # encode 는 이제 전용 함수 사용
    from pyvesc.VESC.messages.getters import GetMcConfRequest  # 결과 확인용
    from pyvesc.VESC.messages.setters import DetectApplyAllFOC  # 마법사 Setter

    # 파서 (결과 확인용)
    from pyvesc.VESC.messages.parser import parse_mc_conf_serialized

    # 패커/SetMcConf 는 이 스크립트에서 직접 사용 안 함 (마법사가 내부 처리)

except ImportError as e:
    print(f"오류: pyvesc import 실패 ({e}).")
    sys.exit(1)
except AttributeError as e:
    print(f"오류: 클래스 속성 문제 ({e}).")
    sys.exit(1)

# --- 설정 ---
SERIAL_PORT = "/dev/cu.usbmodem3041"
BAUD_RATE = 115200
TIMEOUT = 1.0  # 일반 응답용 타임아웃
DETECTION_WIZARD_TIMEOUT = 120.0  # 마법사 완료 대기 시간 (넉넉하게)

# --- 마법사 파라미터 (!!! VESC Tool 값 참고 !!!) ---
MAX_POWER_LOSS = 10.0  # 예시 값 (W) - 사용하는 모터에 맞게 설정!
DETECT_CAN_BUS = False
MIN_CURRENT_IN = 0.0  # VESC 기본값 사용
MAX_CURRENT_IN = 0.0  # VESC 기본값 사용
OPENLOOP_RPM = 0.0  # VESC 기본값 사용
SL_ERPM = 0.0  # VESC 기본값 사용


# --- Helper 함수 정의 ---
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


def get_current_mcconf(ser):
    print("  GET_MCCONF 요청 (설정 확인용)...")
    clear_input_buffer(ser)
    request = encode_request(GetMcConfRequest)
    try:
        ser.write(request)
        time.sleep(TIMEOUT)
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


# --- DetectApplyAllFOC 전용 인코딩 함수 ---
def encode_detect_apply_all_foc(message: DetectApplyAllFOC):
    """DetectApplyAllFOC 메시지를 VESC 통신 패킷으로 수동 인코딩합니다."""
    if not isinstance(message, DetectApplyAllFOC):
        raise TypeError("...")
    if not hasattr(message, "fields") or len(message.fields) != 6:
        raise ValueError("...")

    payload_fmt = ">"
    values_to_pack = []
    try:
        for field_name, fmt_char, scale in message.fields:
            payload_fmt += fmt_char
            value = getattr(message, field_name)
            if fmt_char == "i" and scale:
                value = int(round(float(value) * scale))
            elif fmt_char == "?":
                value = int(bool(value))
            values_to_pack.append(value)

        if len(payload_fmt) - 1 != len(values_to_pack) or len(values_to_pack) != 6:
            raise ValueError("내부 오류: 포맷/값 개수 불일치!")

        # print(f"  [DEBUG] 수동 패킹 시도: fmt='{payload_fmt}', values={values_to_pack}")
        data_payload = struct.pack(payload_fmt, *values_to_pack)
        # print(f"  [DEBUG] 데이터 페이로드 생성 성공 (크기: {len(data_payload)} bytes)")

        command_id_byte = bytes([message.id])
        full_payload = command_id_byte + data_payload
        # print(f"  [DEBUG] frame() 입력 페이로드 생성 (ID: {message.id} + Data) 크기: {len(full_payload)} bytes")

        final_packet = frame(full_payload)
        # print(f"  [DEBUG] 최종 패킷 생성 성공 (크기: {len(final_packet)} bytes)")
        return final_packet

    except AttributeError as e:
        raise AttributeError(f"메시지 객체 필드 '{e}' 없음.")
    except (struct.error, TypeError, ValueError) as e:
        print(f"!!! 패킹 오류: {e}")
        raise


# --- 메인 실행 로직 ---
if __name__ == "__main__":
    vesc_serial = None
    final_config = None  # 최종 확인된 설정을 저장할 변수

    print("=" * 60)
    try:
        print(f"      VESC FOC 모터 감지 마법사 테스트 (ID: {DetectApplyAllFOC.id})")
    except AttributeError:
        print("      VESC FOC 모터 감지 마법사 테스트 (ID: ??)")
    print("=" * 60)
    print("\n!!! [경고] 이 스크립트는 모터 감지를 실행하고 결과를")
    print("             자동으로 적용/저장할 수 있습니다 !!!")
    print("             안전 절차를 반드시 준수하세요! (모터 회전)")
    print("=" * 60)
    print(f"사용될 Max Power Loss: {MAX_POWER_LOSS} W")
    confirm = input("FOC 감지 마법사를 실행하시겠습니까? (y/n): ")
    if confirm.lower() != "y":
        print("실행 취소됨.")
        sys.exit(0)

    try:
        vesc_serial = connect_to_vesc(SERIAL_PORT, BAUD_RATE, TIMEOUT)
        if not vesc_serial:
            sys.exit(1)

        print("\n--- [단계 1] FOC 감지 및 적용 명령 전송 ---")
        wizard_message = DetectApplyAllFOC()
        wizard_message.detect_can = DETECT_CAN_BUS
        wizard_message.max_power_loss = MAX_POWER_LOSS
        wizard_message.min_current_in = MIN_CURRENT_IN
        wizard_message.max_current_in = MAX_CURRENT_IN
        wizard_message.openloop_rpm = OPENLOOP_RPM
        wizard_message.sl_erpm = SL_ERPM

        print(f"  FOC 감지 마법사 명령 (ID: {wizard_message.id}) 준비...")
        print(f"    - Max Power Loss: {MAX_POWER_LOSS:.1f} W")

        try:
            # 전용 인코딩 함수 사용
            packet_wizard = encode_detect_apply_all_foc(wizard_message)

            clear_input_buffer(vesc_serial)
            print("  마법사 명령 전송...")
            bytes_written = vesc_serial.write(packet_wizard)
            print(f"  {bytes_written} 바이트 전송 완료.")
            print(f"\n  VESC 감지 프로세스 진행 중...")
            print(f"  완료 및 응답 대기 (최대 {DETECTION_WIZARD_TIMEOUT}초)...")
            print("  (이 과정에서 모터가 여러 번 움직이거나 소리가 날 수 있습니다)")

            wizard_response = b""
            start_time_wizard = time.time()
            response_received_wizard = False
            while time.time() - start_time_wizard < DETECTION_WIZARD_TIMEOUT:
                if vesc_serial.in_waiting > 0:
                    time.sleep(0.5)
                    # 응답이 완전히 들어올 시간 확보
                    wizard_response += vesc_serial.read(vesc_serial.in_waiting)
                    response_received_wizard = True
                    break
                time.sleep(0.1)

            if not response_received_wizard:
                print("  !!! 오류: 감지 마법사 응답 시간 초과 !!!")
            else:
                print(f"  마법사 응답 수신 ({len(wizard_response)} 바이트).")
                try:
                    payload_wizard, consumed_wizard = unframe(wizard_response)
                    # !!!!! 수정: ID 14 (GET_MCCONF) 응답을 기대 !!!!!
                    if (
                        payload_wizard and payload_wizard[0] == GetMcConfRequest.id
                    ):  # ID 14 확인
                        print(f"  마법사 실행 결과로 GET_MCCONF 응답 (ID: 14) 수신됨.")
                        print(f"  Unframing 성공.")
                        response_data_wizard = payload_wizard[1:]  # ID 제외

                        # !!!!! COMM_GET_MCCONF 파서 사용 !!!!!
                        print("\n--- 감지 및 적용 결과 (GET_MCCONF 응답 분석) ---")
                        final_config = parse_mc_conf_serialized(
                            response_data_wizard
                        )  # 파싱 결과를 final_config 에 저장
                        if final_config and "MCCONF_SIGNATURE" in final_config:
                            print("  >> 성공: 감지 후 설정 파싱 완료.")
                            r = final_config.get("foc_motor_r")
                            l = final_config.get("foc_motor_l")
                            lambda_ = final_config.get("foc_motor_flux_linkage")
                            kp = final_config.get("foc_current_kp")
                            ki = final_config.get("foc_current_ki")
                            print("\n--- 적용된 주요 파라미터 ---")
                            print(
                                f"  Resistance (R) : {r * 1e3:.3f} mOhm"
                                if r is not None
                                else "N/A"
                            )
                            print(
                                f"  Inductance (L) : {l * 1e6:.3f} uH"
                                if l is not None
                                else "N/A"
                            )
                            print(
                                f"  Flux Linkage (L): {lambda_ * 1e3:.3f} mWb"
                                if lambda_ is not None
                                else "N/A"
                            )
                            print(
                                f"  Current Kp       : {kp:.6f}"
                                if kp is not None
                                else "N/A"
                            )
                            print(
                                f"  Current Ki       : {ki:.6f}"
                                if ki is not None
                                else "N/A"
                            )
                            # 감지 성공으로 간주
                            print(
                                "\n  >> VESC 설정이 성공적으로 업데이트 및 적용되었습니다."
                            )
                        else:
                            print("  >> 실패: 수신된 GET_MCCONF 응답 파싱 실패.")
                            final_config = None  # 실패 시 None으로 설정

                    elif payload_wizard:  # ID 14가 아닌 다른 응답
                        print(
                            f"  오류: 예상치 못한 응답 ID 수신 (Expected: {GetMcConfRequest.id}, Got: {payload_wizard[0]})"
                        )
                        # 여기에 실패 처리를 위한 다른 로직 추가 가능 (예: Result Code 파싱 시도)
                    else:
                        print("  오류: 마법사 응답 Unframing 실패.")
                except Exception as e:
                    print(f"  마법사 응답 처리 중 오류 발생: {e}")
                    import traceback

                    traceback.print_exc()

        except Exception as e:
            print(f"!!! FOC 감지 마법사 명령 처리 중 오류: {e} !!!")
            import traceback

            traceback.print_exc()

    except KeyboardInterrupt:
        print("\n사용자에 의해 중단됨.")
    except Exception as e:
        print(f"\n!!! 스크립트 실행 중 오류 발생: {e} !!!")
        import traceback

        traceback.print_exc()
    finally:
        # 마법사 명령은 결과를 자동 적용하므로 기본적으로 복원 불필요
        # 단, 사용자가 원본 복원을 원하거나 실패 시 복원 로직 추가 가능
        print("\n--- 마무리 작업 ---")
        # 필요시 여기에 if not final_config and original_config: 복원 로직 추가
        close_serial_port(vesc_serial)
        print("\n스크립트 종료.")
