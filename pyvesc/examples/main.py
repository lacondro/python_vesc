import serial
import time
import pyvesc
import pprint

# --- 필요한 모듈 임포트 ---
# from pyvesc.VESC.messages.requests import GetMcConfRequest # 경로에 맞게 수정
# from pyvesc.VESC.messages.vesc_parser import parse_mc_conf_serialized # 경로에 맞게 수정
# 위 경로는 실제 pyvesc 설치 경로 또는 프로젝트 구조에 따라 달라집니다.
# 예시를 위해 임시로 같은 폴더에 있다고 가정하고 import:
from pyvesc.VESC.messages import VedderCmd
from pyvesc.protocol.base import VESCMessage
from requests import GetMcConfRequest  # requests.py 파일이 같은 폴더에 있다고 가정
from parser import (
    parse_mc_conf_serialized,
)  # vesc_parser.py 파일이 같은 폴더에 있다고 가정

# --- 설정 및 Helper 함수 (connect_to_vesc, close_serial_port, clear_input_buffer) ---
# (이전 코드와 동일하게 메인 스크립트에 포함시키거나 별도 모듈로 분리)
SERIAL_PORT = "/dev/cu.usbmodem3041"
BAUD_RATE = 115200
TIMEOUT = 1.0
# ... (Helper 함수 정의) ...

# --- 메인 실행 부분 ---
if __name__ == "__main__":
    vesc_serial = None
    try:
        vesc_serial = connect_to_vesc(SERIAL_PORT, BAUD_RATE, TIMEOUT)

        if vesc_serial:
            print("\n--- COMM_GET_MCCONF 요청 및 최종 파싱 시도 (모듈화) ---")
            print("요청 전 입력 버퍼 정리...")
            clear_input_buffer(vesc_serial, wait_time=0.1)

            # 요청 ID 14 사용 (requests.py에서 가져옴)
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
                # print(f"수신된 데이터 (앞 100바이트): {mcconf_buffer[:100]}...") # 필요시 주석 해제

                header_size = 4
                if len(mcconf_buffer) > header_size:
                    response_id = mcconf_buffer[3]
                    print(f"  응답 패킷 ID: {response_id} (0x{response_id:02x})")
                    payload = mcconf_buffer[header_size:]

                    # *** 분리된 파싱 함수 호출 (vesc_parser.py에서 가져옴) ***
                    parsed_motor_config = parse_mc_conf_serialized(payload)

                    if (
                        parsed_motor_config
                        and "MCCONF_SIGNATURE" in parsed_motor_config
                    ):  # 파싱 성공 여부 간단히 확인
                        print("\n--- 최종 파싱된 모터 구성 결과 ---")
                        pprint.pprint(parsed_motor_config)
                        print("---------------------------------")

                        # 값 비교 (선택 사항)
                        # ... (이전과 동일한 값 비교 로직) ...
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
        close_serial_port(vesc_serial)
        print("프로그램 종료.")
