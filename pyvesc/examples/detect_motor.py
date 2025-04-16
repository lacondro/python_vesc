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
    from pyvesc.protocol.packet.codec import *
    from pyvesc.protocol.interface import *  # encode 는 이제 사용 안 함
    from pyvesc.VESC.messages.getters import *  # 결과 확인용
    from pyvesc.VESC.messages.setters import *  # 마법사 Setter
    from pyvesc.VESC.messages.parser import *

    # !!!!! 새로 만든 유틸리티 파일에서 인코딩 함수 import !!!!!
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
DETECTION_WIZARD_TIMEOUT = 120.0

# --- 마법사 파라미터 ---
MAX_POWER_LOSS = 100.0
DETECT_CAN_BUS = False
MIN_CURRENT_IN = 0.0
MAX_CURRENT_IN = 0.0
OPENLOOP_RPM = 0.0
SL_ERPM = 0.0


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
            # !!!!! import된 인코딩 헬퍼 사용 !!!!!
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
                    # 응답이 ID 14 (GET_MCCONF) 로 온다고 가정
                    if payload_wizard and payload_wizard[0] == GetMcConfRequest.id:
                        print(f"  마법사 실행 결과로 GET_MCCONF 응답 (ID: 14) 수신됨.")
                        print(f"  Unframing 성공.")
                        response_data_wizard = payload_wizard[1:]

                        print("\n--- 감지 및 적용 결과 (GET_MCCONF 응답 분석) ---")
                        final_config = parse_mc_conf_serialized(response_data_wizard)
                        if final_config and "MCCONF_SIGNATURE" in final_config:
                            print("  >> 성공: 감지 후 설정 파싱 완료.")
                            r = final_config.get("foc_motor_r")
                            l = final_config.get("foc_motor_l")
                            lambda_ = final_config.get("foc_motor_flux_linkage")
                            kp = final_config.get("foc_current_kp")
                            ki = final_config.get("foc_current_ki")
                            observer_gain = final_config.get(
                                "foc_observer_gain"
                            )  # Observer Gain 추가
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
                            print(
                                f"  Observer Gain    : {observer_gain:.2f}"
                                if observer_gain is not None
                                else "N/A"
                            )  # Observer Gain 출력
                            print(
                                "\n  >> VESC 설정이 성공적으로 업데이트 및 적용되었습니다."
                            )
                        else:
                            print("  >> 실패: 수신된 GET_MCCONF 응답 파싱 실패.")
                            final_config = None

                    elif payload_wizard:
                        print(
                            f"  오류: 예상치 못한 응답 ID 수신 (Expected: {GetMcConfRequest.id}, Got: {payload_wizard[0]})"
                        )
                        # 여기에 실패 시 Result Code 파싱 시도 로직 추가 가능
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
        print("\n--- 마무리 작업 ---")
        # 마법사 명령은 결과를 자동 적용하므로 별도 복원 불필요
        # (단, 실패 시 복원 로직은 필요할 수 있음)
        close_serial_port(vesc_serial)
        print("\n스크립트 종료.")
