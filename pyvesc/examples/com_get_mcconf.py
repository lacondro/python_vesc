import sys
import os  # sys, os 임포트 추가 (경로 설정 등에 필요할 수 있음)
import serial
import time
import struct  # 필요할 수 있으므로 유지
import pprint
import pyvesc  # 사용자 정의 pyvesc 경로 설정 필요 가능성

# !!!!! 필요한 메시지 및 파서 함수 import !!!!!
try:
    # GetMcConfRequest 는 getters 에 정의되어 있다고 가정
    from pyvesc.VESC.messages.getters import GetMcConfRequest

    # !!!!! parser.py 에서 파서 함수를 import !!!!!
    # 실제 parser.py 위치에 맞게 경로 수정 필요
    # 예: 프로젝트 루트에 parser.py 가 있다면 from parser import ...
    # 예: pyvesc 라이브러리 구조 내에 있다면 아래 경로 사용 가능성
    from pyvesc.VESC.messages.parser import parse_mc_conf_serialized
    from pyvesc.protocol.interface import encode_request  # GET 요청 인코더
    from pyvesc.protocol.packet.codec import unframe  # 응답 디코더 (CRC 검증 포함)
except ImportError as e:
    print(f"오류: 필요한 pyvesc 모듈/클래스/함수 import 실패.")
    print(f"    ({e})")
    print(
        "    라이브러리 경로 및 파일 정의(GetMcConfRequest, parse_mc_conf_serialized) 확인 필요"
    )
    sys.exit(1)
except AttributeError as e:
    print(f"오류: 클래스 속성 관련 문제 ({e}).")
    sys.exit(1)


# --- 설정 ---
SERIAL_PORT = "/dev/cu.usbmodem3041"  # 실제 포트로 변경
BAUD_RATE = 115200
TIMEOUT = 1.0


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


# --- 메인 실행 부분 ---
if __name__ == "__main__":
    vesc_serial = None
    try:
        vesc_serial = connect_to_vesc(SERIAL_PORT, BAUD_RATE, TIMEOUT)

        if vesc_serial:
            print("\n--- COMM_GET_MCCONF 요청 및 파싱 (외부 파서 사용) ---")

            print("요청 전 입력 버퍼 정리...")
            clear_input_buffer(vesc_serial, wait_time=0.1)

            # encode_request 사용 (GetMcConfRequest는 fields=[] 가정)
            request_packet = encode_request(GetMcConfRequest)  # pyvesc의 인코더 사용
            print(f"COMM_GET_MCCONF (ID {GetMcConfRequest.id}) 요청 전송 중...")
            vesc_serial.write(request_packet)
            print("요청 전송 완료.")

            response_wait_time = 1.0
            print(f"{response_wait_time}초 동안 응답 대기...")
            time.sleep(response_wait_time)

            read_size = 4096
            print(f"최대 {read_size} 바이트 응답 읽기 시도...")
            response_buffer = vesc_serial.read(
                read_size
            )  # 변수명 변경 mcconf_buffer -> response_buffer

            if response_buffer:
                print(f"\n수신 성공: {len(response_buffer)} 바이트.")
                # print(f"수신 데이터 (앞 100바이트): {response_buffer[:100]}...") # 필요시 주석 해제

                # !!!!! 응답 패킷 처리: unframe 사용 권장 !!!!!
                parsed_motor_config = None
                try:
                    # frame/unframe 함수 import 확인
                    from pyvesc.protocol.packet.codec import unframe

                    payload, consumed = unframe(response_buffer)  # CRC 검증 포함

                    if payload and payload[0] == GetMcConfRequest.id:
                        print(
                            f"  Unframing 성공 (ID: {GetMcConfRequest.id}). 파싱 시도..."
                        )
                        actual_data_payload = payload[1:]  # ID 제외
                        # !!!!! import 된 파서 함수 호출 !!!!!
                        parsed_motor_config = parse_mc_conf_serialized(
                            actual_data_payload
                        )
                    elif payload:
                        print(
                            f"  오류: 예상치 못한 응답 ID 수신 (Expected: {GetMcConfRequest.id}, Got: {payload[0]})"
                        )
                    else:
                        print(
                            f"  오류: Unframing 실패 또는 유효한 패킷 없음 (소비: {consumed})."
                        )

                except ImportError:
                    print(
                        "  경고: 'unframe' 함수 import 불가. 수동 파싱 시도 (CRC 미검증)."
                    )
                    # --- 수동 파싱 대체 로직 ---
                    header_size = 4  # 예시
                    if (
                        len(response_buffer) > header_size + 2
                        and response_buffer[3] == GetMcConfRequest.id
                    ):
                        payload_manual = response_buffer[
                            header_size:-2
                        ]  # CRC 제외 가정
                        parsed_motor_config = parse_mc_conf_serialized(
                            payload_manual
                        )  # import된 파서 사용
                    else:
                        print("  오류: 수신 데이터 형식 오류 (수동).")
                except Exception as e:
                    print(f"  Unframing/파싱 중 오류 발생: {e}")
                    import traceback

                    traceback.print_exc()

                # --- 파싱 결과 처리 ---
                if parsed_motor_config:
                    print("\n--- 파싱된 모터 구성 결과 ---")
                    pprint.pprint(parsed_motor_config)
                    print("---------------------------------")

                    # (값 비교 로직은 필요하면 유지)
                    # ...

                else:
                    print(
                        "\n오류: 모터 구성 데이터 파싱에 실패했거나 유효하지 않습니다."
                    )

            else:
                print("\n오류: COMM_GET_MCCONF 응답 없음 (타임아웃?).")

        else:
            print("\nVESC 연결 실패.")

    except KeyboardInterrupt:
        print("\n사용자에 의해 프로그램 중단됨.")
    finally:
        close_serial_port(vesc_serial)
        print("프로그램 종료.")
