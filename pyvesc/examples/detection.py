# detect_motor.py (time.sleep 사용, 업데이트된 설정 읽기 방식)

import sys
import os
import serial
import time
import pyvesc  # 설치된 pyvesc 라이브러리 사용
import pprint
import struct
import traceback

# --- pyvesc 라이브러리 경로 설정 (필요시) ---
# (만약 editable 설치를 했다면 이 부분은 필요 없을 수도 있음)
current_file_path = os.path.abspath(__file__)
examples_dir = os.path.dirname(current_file_path)
project_root = os.path.dirname(examples_dir)
pyvesc_dir_name = "pyvesc"  # 실제 폴더 이름 확인
pyvesc_dir_path = os.path.join(project_root, pyvesc_dir_name)
if project_root not in sys.path:
    sys.path.insert(0, project_root)
    print(f"Added to sys.path: {project_root}")

# --- 필요한 모듈 임포트 ---
try:
    # 설정 읽기 요청 (getters.py에서, 커스텀 ID 14 가정)
    from pyvesc.VESC.messages.getters import GetMcConfRequest

    # 감지 명령 요청 (setters.py에서, ID 25, 26)
    from pyvesc.VESC.messages.setters import DetectMotorRL, DetectMotorFluxLinkage

    # 설정 파서 함수 (vesc_parser.py에서)
    from pyvesc.VESC.messages.parser import parse_mc_conf_serialized

    # 메시지 베이스 클래스 및 인코딩 함수
    from pyvesc.protocol.base import VESCMessage
    from pyvesc import encode, encode_request  # encode 함수 임포트 추가
except ImportError as e:
    print(f"Import Error: {e}")
    print("Failed to import necessary modules from pyvesc.")
    print(
        "Ensure pyvesc is correctly installed (e.g., 'pip install -e .' in parent dir)"
    )
    print(
        "and required files/classes exist (GetCustomMcConfRequest in getters, DetectMotorRL/FluxLinkage in setters, parse_mc_conf_serialized in vesc_parser)."
    )
    sys.exit(1)
except NameError as e:
    print(
        f"NameError during import: {e}. Trying to import encode/encode_request from pyvesc.protocol"
    )
    try:
        from pyvesc.protocol import encode, encode_request
    except ImportError:
        print("Failed to import encode/encode_request from pyvesc.protocol as well.")
        sys.exit(1)


# --- 설정 ---
SERIAL_PORT = "/dev/cu.usbmodem3041"
BAUD_RATE = 115200
TIMEOUT = 1.0
# *** 감지 대기 시간 설정 (중요!) ***
RL_DETECTION_WAIT_TIME = 10  # R/L 감지 후 VESC 내부 처리 시간 + 통신 시간 고려
FLUX_DETECTION_WAIT_TIME = 20  # 자속 감지 후 VESC 내부 처리 시간 + 통신 시간 고려
# *** 자속 감지 파라미터 (중요!) ***
FLUX_DETECTION_CURRENT = 10.0  # 예: 감지 전류 (A) - 모터에 맞게 조정
FLUX_DETECTION_MIN_RPM = 2000.0  # 예: 최소 RPM - 모터에 맞게 조정
FLUX_DETECTION_DUTY = 0.3  # 예: 듀티 사이클 - 모터에 맞게 조정


# --- Helper 함수 정의 ---
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
            # 감지 후 모터 정지 명령 추가 (선택 사항)
            # from pyvesc.VESC.messages.setters import SetCurrent
            # ser.write(encode(SetCurrent(0)))
            # time.sleep(0.1)
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


# ===> get_and_parse_mc_conf 함수는 그대로 사용 <===
def get_and_parse_mc_conf(ser):
    """Helper 함수: 모터 설정을 요청하고 파싱하여 딕셔너리로 반환"""
    print("\n  요청: 모터 설정 읽기 (ID 14)")
    clear_input_buffer(ser, 0.1)
    request_packet = encode_request(GetMcConfRequest)  # 요청 클래스 사용
    ser.write(request_packet)
    time.sleep(1.0)  # 응답 대기
    read_buffer = ser.read(4096)

    if read_buffer:
        # print(f"  수신: {len(read_buffer)} 바이트")
        header_size = 4
        if len(read_buffer) > header_size:
            response_id = read_buffer[3]
            if response_id == 14:  # 예상 응답 ID 확인
                payload = read_buffer[header_size:]
                try:
                    parsed_config = parse_mc_conf_serialized(payload)
                    if parsed_config and "MCCONF_SIGNATURE" in parsed_config:
                        # print("  설정 읽기 및 파싱 성공.") # 로그 줄이기
                        return parsed_config
                    else:
                        print("  오류: 파싱 함수가 유효한 결과를 반환하지 않음.")
                        return None
                except Exception as e:
                    print(f"  파싱 중 오류 발생: {e}")
                    traceback.print_exc()
                    return None
            else:
                print(f"  오류: 예상치 못한 응답 ID 수신 ({response_id})")
                return None
        else:
            print("  오류: 너무 짧은 응답 수신")
            return None
    else:
        print("  오류: 설정 읽기 응답 없음")
        return None


# -------------------------------------------


# --- 감지 실행 메인 로직 ---
if __name__ == "__main__":
    vesc_serial = None
    print("===== VESC 모터 파라미터 감지 스크립트 (업데이트된 설정 읽기 방식) =====")
    print("경고: 이 스크립트는 모터를 움직이거나 회전시킬 수 있습니다.")
    print("      안전한 환경에서만 실행하고, 모터가 단단히 고정되었는지 확인하세요.")

    try:
        vesc_serial = connect_to_vesc(SERIAL_PORT, BAUD_RATE, TIMEOUT)

        if vesc_serial:

            print("\n어떤 감지를 수행하시겠습니까?")
            print("1: R/L 감지")
            print("2: 자속 연계량 감지 (모터 회전!)")
            print("3: R/L 및 자속 모두 감지 (순차적 실행)")
            choice = input("선택 (1/2/3): ")

            run_rl = False
            run_flux = False
            if choice == "1":
                run_rl = True
            elif choice == "2":
                run_flux = True
            elif choice == "3":
                run_rl = True
                run_flux = True
            else:
                print("잘못된 선택.")
                sys.exit(1)

            config_before = None
            config_current = None  # 현재 VESC 설정을 추적하기 위한 변수

            # 감지 전 설정 읽기
            if run_rl or run_flux:
                print("\n--- 감지 전 모터 설정 읽기 시도 ---")
                config_before = get_and_parse_mc_conf(vesc_serial)
                if config_before:
                    print("  감지 전 설정 읽기 성공.")
                    config_current = config_before  # 초기 설정값 저장
                    print(f"    (초기 R: {config_current.get('foc_motor_r', 'N/A')})")
                    print(f"    (초기 L: {config_current.get('foc_motor_l', 'N/A')})")
                    print(
                        f"    (초기 Flux: {config_current.get('foc_motor_flux_linkage', 'N/A')})"
                    )
                else:
                    print(
                        "  감지 전 설정을 읽지 못했습니다. 감지를 진행할 수 없습니다."
                    )
                    sys.exit(1)  # 초기 설정 없으면 중단

            # --- R/L 감지 실행 ---
            if run_rl:
                print("\n--- 모터 R/L 감지 요청 (ID 25) ---")
                input(
                    f">>> 경고: 모터가 살짝 움직일 수 있습니다. 안전 확인 후 Enter..."
                )
                request_rl = encode_request(DetectMotorRL)  # 파라미터 없음
                clear_input_buffer(vesc_serial, 0.1)
                vesc_serial.write(request_rl)
                print(f"R/L 감지 요청 전송됨. 약 {RL_DETECTION_WAIT_TIME}초 대기...")
                time.sleep(RL_DETECTION_WAIT_TIME)  # 감지 및 VESC 내부 처리 대기
                print("R/L 감지 대기 완료.")

                # ===> 감지 후 설정 다시 읽기 <===
                print("\n--- R/L 감지 후 모터 설정 읽기 ---")
                config_after_rl = get_and_parse_mc_conf(vesc_serial)

                if config_after_rl:
                    print("\n--- R/L 감지 결과 (설정값 확인) ---")
                    print(f"  foc_motor_r: {config_after_rl.get('foc_motor_r', 'N/A')}")
                    print(f"  foc_motor_l: {config_after_rl.get('foc_motor_l', 'N/A')}")
                    if config_current:  # 이전 값과 비교
                        print(
                            f"    (이전 R: {config_current.get('foc_motor_r', 'N/A')})"
                        )
                        print(
                            f"    (이전 L: {config_current.get('foc_motor_l', 'N/A')})"
                        )
                    print("--------------------------------")
                    config_current = config_after_rl  # 현재 설정 업데이트
                else:
                    print("오류: R/L 감지 후 설정을 읽지 못했습니다.")
                    run_flux = False  # R/L 실패 시 자속 감지 건너뛰기

            # --- 자속 감지 실행 ---
            if run_flux:
                # 자속 감지에 필요한 저항값 확인
                if (
                    config_current
                    and "foc_motor_r" in config_current
                    and config_current["foc_motor_r"] is not None
                ):
                    measured_resistance = config_current["foc_motor_r"]
                    print(f"\n--- 모터 자속 연계량 감지 요청 (ID 26) ---")
                    print(
                        f"  사용 파라미터: Current={FLUX_DETECTION_CURRENT}A, Min RPM={FLUX_DETECTION_MIN_RPM}, Duty={FLUX_DETECTION_DUTY}, R={measured_resistance:.6f} Ohm"
                    )
                    input(f">>> !!!경고!!! 모터가 회전합니다! 안전 확인 후 Enter...")

                    try:
                        # 파라미터와 함께 요청 객체 생성
                        request_flux_obj = DetectMotorFluxLinkage(
                            current=FLUX_DETECTION_CURRENT,
                            min_rpm=FLUX_DETECTION_MIN_RPM,
                            duty=FLUX_DETECTION_DUTY,
                            resistance=measured_resistance,
                        )
                        # encode() 사용하여 페이로드 포함하여 인코딩
                        request_flux_packet = encode(request_flux_obj)

                        clear_input_buffer(vesc_serial, 0.1)
                        vesc_serial.write(request_flux_packet)
                        print(
                            f"자속 감지 요청 전송됨. 약 {FLUX_DETECTION_WAIT_TIME}초 대기..."
                        )
                        time.sleep(
                            FLUX_DETECTION_WAIT_TIME
                        )  # 감지 및 VESC 내부 처리 대기
                        print("자속 감지 대기 완료.")

                        # ===> 감지 후 설정 다시 읽기 <===
                        print("\n--- 자속 감지 후 모터 설정 읽기 ---")
                        config_after_flux = get_and_parse_mc_conf(vesc_serial)

                        if config_after_flux:
                            print("\n--- 자속 감지 결과 (설정값 확인) ---")
                            print(
                                f"  foc_motor_flux_linkage: {config_after_flux.get('foc_motor_flux_linkage', 'N/A')}"
                            )
                            print(
                                f"  foc_observer_gain: {config_after_flux.get('foc_observer_gain', 'N/A')}"
                            )  # 옵저버 게인도 변경될 수 있음
                            if config_current:  # 이전 값과 비교
                                print(
                                    f"    (이전 Flux Linkage: {config_current.get('foc_motor_flux_linkage', 'N/A')})"
                                )
                                print(
                                    f"    (이전 Observer Gain: {config_current.get('foc_observer_gain', 'N/A')})"
                                )
                            print("----------------------------")
                            print(
                                "참고: 감지된 값(R, L, Flux Linkage)을 VESC 설정에 영구 저장하려면"
                            )
                            print(
                                "      별도로 COMM_SET_MCCONF 명령을 사용해야 합니다."
                            )
                        else:
                            print("오류: 자속 감지 후 설정을 읽지 못했습니다.")

                    except Exception as e:
                        print(f"\n자속 감지 요청/처리 중 오류 발생: {e}")
                        traceback.print_exc()
                else:
                    print(
                        "\n오류: 자속 감지를 위해 필요한 저항(R) 값이 현재 설정에 없습니다."
                    )

            print("\n모터 감지 스크립트 완료.")

        else:
            print("\nVESC 연결 실패.")

    except KeyboardInterrupt:
        print("\n사용자에 의해 프로그램 중단됨.")
    except NameError as e:
        print(f"\nNameError 발생: {e}")
    except Exception as e:
        print(f"\n스크립트 실행 중 예상치 못한 오류 발생: {e}")
        traceback.print_exc()
    finally:
        if "vesc_serial" in locals() or "vesc_serial" in globals():
            close_serial_port(vesc_serial)
        else:
            print("시리얼 객체가 생성되지 않아 닫을 수 없습니다.")
        print("프로그램 종료.")
