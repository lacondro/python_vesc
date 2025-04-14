import serial
import time
import struct
import pprint
import pyvesc
from pyvesc.VESC.messages import VedderCmd
from pyvesc.protocol.base import VESCMessage

# --- 설정 (이전과 동일) ---
SERIAL_PORT = "/dev/cu.usbmodem3041"
BAUD_RATE = 115200
TIMEOUT = 1.0
# -----------------------------


# --- Helper 함수 (이전과 동일) ---
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


# -----------------------------------


# --- COMM_GET_MCCONF 요청 메시지 정의 (ID 14 - 커스텀) ---
# 또는 표준 ID 12를 사용하고 응답 ID 14를 기다릴 수도 있습니다.
# 우선은 요청 ID 14를 사용해봅니다.
class GetMcConfCustom(metaclass=VESCMessage):
    id = VedderCmd.COMM_GET_MCCONF  # VESC 6.05 커스텀 enum 기준
    fields = []


# -----------------------------------


# --- mc_configuration 파싱 함수 (confgenerator_serialize_mcconf 기반) ---
def parse_mc_conf_serialized(payload_buffer):
    """
    VESC 6.05 펌웨어의 confgenerator_serialize_mcconf 함수 로직에 맞춰 파싱합니다.
    """
    parsed_config = {}
    offset = 0
    buffer_len = len(payload_buffer)
    print(
        f"mc_configuration (Serialized) 파싱 시작 (페이로드 크기: {buffer_len} 바이트)..."
    )
    print(
        f"페이로드 시작 (Hex): {' '.join(f'{b:02x}' for b in payload_buffer[:32])}..."
    )

    # 내부 헬퍼 함수
    def unpack_and_advance(fmt, field_name, scale=None):
        nonlocal offset
        # ... (이전 unpack_helper 로직과 동일하게 오류 처리 포함) ...
        try:
            full_fmt = ">" + fmt
            size = struct.calcsize(full_fmt)
            if offset + size > buffer_len:
                if field_name != "crc":
                    print(
                        f"\n오류: 필드 '{field_name}' 파싱 중 버퍼 길이 초과 (필요: {size}, 남음: {buffer_len - offset}, offset={offset})"
                    )
                    raise IndexError(
                        f"Buffer length exceeded parsing field '{field_name}'"
                    )
                else:
                    print(f"\n경고: CRC 필드 '{field_name}' 파싱 중 버퍼 길이 부족")
                    return None
            value = struct.unpack_from(full_fmt, payload_buffer, offset)[0]
            if scale is not None and scale != 0:
                value = float(value) / scale
            parsed_config[field_name] = value
            offset += size
            return value
        except (struct.error, IndexError) as e:
            print(
                f"\n오류: 필드 '{field_name}' 파싱 중 오류 발생 (offset={offset}, fmt='{full_fmt}')"
            )
            print(f"  Error: {e}")
            raise

    try:
        # --- confgenerator_serialize_mcconf 순서대로 파싱 ---
        # 1. Signature (uint32_t) - 읽고 저장하거나 건너뜀
        signature = unpack_and_advance("I", "MCCONF_SIGNATURE")
        print(f"  Signature: {signature} (0x{signature:X})")

        # 2. Enum/bool (1 byte)
        unpack_and_advance("B", "pwm_mode")
        unpack_and_advance("B", "comm_mode")  # <<< 확인 필요 (0 기대)
        unpack_and_advance("B", "motor_type")
        unpack_and_advance("B", "sensor_mode")
        # 3. Limits (float32 / float16)
        unpack_and_advance("f", "l_current_max")  # <<< 확인 필요 (48.94 기대)
        unpack_and_advance("f", "l_current_min")
        unpack_and_advance("f", "l_in_current_max")
        unpack_and_advance("f", "l_in_current_min")
        unpack_and_advance("h", "l_in_current_map_start", scale=10000)
        unpack_and_advance("h", "l_in_current_map_filter", scale=10000)
        unpack_and_advance("f", "l_abs_current_max")
        unpack_and_advance("f", "l_min_erpm")
        unpack_and_advance("f", "l_max_erpm")
        unpack_and_advance("h", "l_erpm_start", scale=10000)
        unpack_and_advance("f", "l_max_erpm_fbrake")
        unpack_and_advance("f", "l_max_erpm_fbrake_cc")
        unpack_and_advance("h", "l_min_vin", scale=10)
        unpack_and_advance("h", "l_max_vin", scale=10)
        unpack_and_advance("h", "l_battery_cut_start", scale=10)
        unpack_and_advance("h", "l_battery_cut_end", scale=10)
        unpack_and_advance("h", "l_battery_regen_cut_start", scale=10)
        unpack_and_advance("h", "l_battery_regen_cut_end", scale=10)
        unpack_and_advance("?", "l_slow_abs_current")  # bool
        # 온도 관련: float 이지만 uint8_t 로 캐스팅되어 전송됨
        unpack_and_advance("B", "l_temp_fet_start")
        unpack_and_advance("B", "l_temp_fet_end")
        unpack_and_advance("B", "l_temp_motor_start")
        unpack_and_advance("B", "l_temp_motor_end")
        unpack_and_advance("h", "l_temp_accel_dec", scale=10000)
        unpack_and_advance("h", "l_min_duty", scale=10000)
        unpack_and_advance("h", "l_max_duty", scale=10000)
        unpack_and_advance("f", "l_watt_max")
        unpack_and_advance("f", "l_watt_min")
        unpack_and_advance("h", "l_current_max_scale", scale=10000)
        unpack_and_advance("h", "l_current_min_scale", scale=10000)
        unpack_and_advance("h", "l_duty_start", scale=10000)
        # Sensorless
        unpack_and_advance("f", "sl_min_erpm")
        unpack_and_advance("f", "sl_min_erpm_cycle_int_limit")
        unpack_and_advance("f", "sl_max_fullbreak_current_dir_change")
        unpack_and_advance("h", "sl_cycle_int_limit", scale=10)
        unpack_and_advance("h", "sl_phase_advance_at_br", scale=10000)
        unpack_and_advance("f", "sl_cycle_int_rpm_br")
        unpack_and_advance("f", "sl_bemf_coupling_k")
        # Hall table: int8_t 배열이지만 uint8_t로 캐스팅되어 전송됨 -> 'B' 또는 'b'로 읽기
        fmt = ">8b"
        size = struct.calcsize(fmt)  # 'b' (signed) 로 읽어봄
        if offset + size > buffer_len:
            raise IndexError("Buffer length exceeded for hall_table")
        hall_table = struct.unpack_from(fmt, payload_buffer, offset)
        parsed_config["hall_table"] = list(hall_table)
        offset += size
        unpack_and_advance("f", "hall_sl_erpm")
        # FOC
        unpack_and_advance("f", "foc_current_kp")  # <<< 확인 필요 (0.0532 기대)
        unpack_and_advance("f", "foc_current_ki")  # <<< 확인 필요 (33.40 기대)
        unpack_and_advance("f", "foc_f_zv")
        unpack_and_advance("f", "foc_dt_us")
        unpack_and_advance("?", "foc_encoder_inverted")  # bool
        unpack_and_advance("f", "foc_encoder_offset")
        unpack_and_advance("f", "foc_encoder_ratio")
        unpack_and_advance("B", "foc_sensor_mode")  # Enum
        unpack_and_advance("f", "foc_pll_kp")
        unpack_and_advance("f", "foc_pll_ki")
        unpack_and_advance("f", "foc_motor_l")
        unpack_and_advance("f", "foc_motor_ld_lq_diff")
        unpack_and_advance("f", "foc_motor_r")
        unpack_and_advance("f", "foc_motor_flux_linkage")
        unpack_and_advance("f", "foc_observer_gain")
        unpack_and_advance("f", "foc_observer_gain_slow")
        unpack_and_advance("h", "foc_observer_offset", scale=1000)
        unpack_and_advance("f", "foc_duty_dowmramp_kp")
        unpack_and_advance("f", "foc_duty_dowmramp_ki")
        unpack_and_advance("h", "foc_start_curr_dec", scale=10000)
        unpack_and_advance("f", "foc_start_curr_dec_rpm")
        unpack_and_advance("f", "foc_openloop_rpm")
        unpack_and_advance("h", "foc_openloop_rpm_low", scale=1000)
        unpack_and_advance("h", "foc_d_gain_scale_start", scale=1000)
        unpack_and_advance("h", "foc_d_gain_scale_max_mod", scale=1000)
        unpack_and_advance("h", "foc_sl_openloop_hyst", scale=100)
        unpack_and_advance("h", "foc_sl_openloop_time_lock", scale=100)
        unpack_and_advance("h", "foc_sl_openloop_time_ramp", scale=100)
        unpack_and_advance("h", "foc_sl_openloop_time", scale=100)
        unpack_and_advance("h", "foc_sl_openloop_boost_q", scale=100)
        unpack_and_advance("h", "foc_sl_openloop_max_q", scale=100)
        # FOC Hall table (uint8_t[8])
        fmt = ">8B"
        size = struct.calcsize(fmt)
        if offset + size > buffer_len:
            raise IndexError("Buffer length exceeded for foc_hall_table")
        foc_hall_table = struct.unpack_from(fmt, payload_buffer, offset)
        parsed_config["foc_hall_table"] = list(foc_hall_table)
        offset += size
        unpack_and_advance("f", "foc_hall_interp_erpm")
        unpack_and_advance("f", "foc_sl_erpm_start")
        unpack_and_advance("f", "foc_sl_erpm")
        unpack_and_advance("B", "foc_control_sample_mode")  # Enum
        unpack_and_advance("B", "foc_current_sample_mode")  # Enum
        unpack_and_advance("B", "foc_sat_comp_mode")  # Enum
        unpack_and_advance("h", "foc_sat_comp", scale=1000)
        unpack_and_advance("?", "foc_temp_comp")  # bool
        unpack_and_advance("h", "foc_temp_comp_base_temp", scale=100)
        unpack_and_advance(
            "h", "foc_current_filter_const", scale=10000
        )  # <<< 확인 필요 (0.1 기대)
        unpack_and_advance("B", "foc_cc_decoupling")  # Enum <<< 확인 필요 (0 기대)
        unpack_and_advance("B", "foc_observer_type")  # Enum
        unpack_and_advance("h", "foc_hfi_voltage_start", scale=10)
        unpack_and_advance("h", "foc_hfi_voltage_run", scale=10)
        unpack_and_advance("h", "foc_hfi_voltage_max", scale=10)
        unpack_and_advance("h", "foc_hfi_gain", scale=1000)
        unpack_and_advance("h", "foc_hfi_max_err", scale=1000)
        unpack_and_advance("h", "foc_hfi_hyst", scale=100)
        unpack_and_advance("f", "foc_sl_erpm_hfi")
        unpack_and_advance("H", "foc_hfi_start_samples")  # uint16_t
        unpack_and_advance("f", "foc_hfi_obs_ovr_sec")
        unpack_and_advance("B", "foc_hfi_samples")  # Enum
        unpack_and_advance("?", "foc_offsets_cal_on_boot")  # bool
        fmt = ">3f"
        size = struct.calcsize(fmt)
        if offset + size > buffer_len:
            raise IndexError("Buffer length exceeded for foc_offsets_current")
        foc_offsets_current = struct.unpack_from(fmt, payload_buffer, offset)
        parsed_config["foc_offsets_current"] = list(foc_offsets_current)
        offset += size
        fmt = ">3h"
        size = struct.calcsize(fmt)
        if offset + size > buffer_len:
            raise IndexError("Buffer length exceeded for foc_offsets_voltage")
        foc_offsets_voltage_raw = struct.unpack_from(fmt, payload_buffer, offset)
        parsed_config["foc_offsets_voltage"] = [
            v / 10000.0 for v in foc_offsets_voltage_raw
        ]
        offset += size
        fmt = ">3h"
        size = struct.calcsize(fmt)
        if offset + size > buffer_len:
            raise IndexError("Buffer length exceeded for foc_offsets_voltage_undriven")
        foc_offsets_voltage_undriven_raw = struct.unpack_from(
            fmt, payload_buffer, offset
        )
        parsed_config["foc_offsets_voltage_undriven"] = [
            v / 10000.0 for v in foc_offsets_voltage_undriven_raw
        ]
        offset += size
        unpack_and_advance("?", "foc_phase_filter_enable")  # bool
        unpack_and_advance("?", "foc_phase_filter_disable_fault")  # bool
        unpack_and_advance("f", "foc_phase_filter_max_erpm")
        unpack_and_advance("B", "foc_mtpa_mode")  # Enum
        unpack_and_advance("f", "foc_fw_current_max")
        unpack_and_advance("h", "foc_fw_duty_start", scale=10000)
        unpack_and_advance("h", "foc_fw_ramp_time", scale=1000)
        unpack_and_advance("h", "foc_fw_q_current_factor", scale=10000)
        unpack_and_advance("B", "foc_speed_soure")  # Enum (typo in C code likely)
        unpack_and_advance("?", "foc_short_ls_on_zero_duty")  # bool
        unpack_and_advance("B", "sp_pid_loop_rate")  # Enum
        unpack_and_advance("f", "s_pid_kp")
        unpack_and_advance("f", "s_pid_ki")
        unpack_and_advance("f", "s_pid_kd")
        unpack_and_advance("h", "s_pid_kd_filter", scale=10000)
        unpack_and_advance("f", "s_pid_min_erpm")
        unpack_and_advance("?", "s_pid_allow_braking")  # bool
        unpack_and_advance("f", "s_pid_ramp_erpms_s")
        unpack_and_advance("B", "s_pid_speed_source")  # Enum
        unpack_and_advance("f", "p_pid_kp")
        unpack_and_advance("f", "p_pid_ki")
        unpack_and_advance("f", "p_pid_kd")
        unpack_and_advance("f", "p_pid_kd_proc")
        unpack_and_advance("h", "p_pid_kd_filter", scale=10000)  # float16
        unpack_and_advance("f", "p_pid_ang_div")
        unpack_and_advance("h", "p_pid_gain_dec_angle", scale=10)
        unpack_and_advance("f", "p_pid_offset")
        unpack_and_advance("h", "cc_startup_boost_duty", scale=10000)
        unpack_and_advance("f", "cc_min_current")
        unpack_and_advance("f", "cc_gain")
        unpack_and_advance("h", "cc_ramp_step_max", scale=10000)
        unpack_and_advance("i", "m_fault_stop_time_ms")  # int32_t
        unpack_and_advance("h", "m_duty_ramp_step", scale=10000)
        unpack_and_advance("f", "m_current_backoff_gain")
        unpack_and_advance("I", "m_encoder_counts")  # uint32_t
        unpack_and_advance("h", "m_encoder_sin_amp", scale=1000)
        unpack_and_advance("h", "m_encoder_cos_amp", scale=1000)
        unpack_and_advance("h", "m_encoder_sin_offset", scale=1000)
        unpack_and_advance("h", "m_encoder_cos_offset", scale=1000)
        unpack_and_advance("h", "m_encoder_sincos_filter_constant", scale=1000)
        unpack_and_advance("h", "m_encoder_sincos_phase_correction", scale=1000)
        unpack_and_advance("B", "m_sensor_port_mode")  # Enum
        unpack_and_advance("?", "m_invert_direction")  # bool
        unpack_and_advance("B", "m_drv8301_oc_mode")  # Enum
        unpack_and_advance("B", "m_drv8301_oc_adj")  # uint8? (캐스팅됨)
        unpack_and_advance("f", "m_bldc_f_sw_min")
        unpack_and_advance("f", "m_bldc_f_sw_max")
        unpack_and_advance("f", "m_dc_f_sw")
        unpack_and_advance("f", "m_ntc_motor_beta")
        unpack_and_advance("B", "m_out_aux_mode")  # Enum
        unpack_and_advance("B", "m_motor_temp_sens_type")  # Enum
        unpack_and_advance("f", "m_ptc_motor_coeff")
        unpack_and_advance(
            "h", "m_ntcx_ptcx_res", scale=0.1
        )  # float16, scale=0.1 이상함? 확인필요
        unpack_and_advance("h", "m_ntcx_ptcx_temp_base", scale=10)
        unpack_and_advance("B", "m_hall_extra_samples")  # uint8? (캐스팅됨)
        unpack_and_advance("B", "m_batt_filter_const")  # uint8? (캐스팅됨)
        unpack_and_advance("B", "si_motor_poles")  # uint8_t
        unpack_and_advance("f", "si_gear_ratio")
        unpack_and_advance("f", "si_wheel_diameter")
        unpack_and_advance("B", "si_battery_type")  # Enum
        unpack_and_advance("B", "si_battery_cells")  # uint8? (캐스팅됨)
        unpack_and_advance("f", "si_battery_ah")
        unpack_and_advance("f", "si_motor_nl_current")
        # BMS Configuration
        unpack_and_advance("B", "bms.type")
        unpack_and_advance("B", "bms.limit_mode")
        unpack_and_advance("B", "bms.t_limit_start")  # uint8? (캐스팅됨)
        unpack_and_advance("B", "bms.t_limit_end")  # uint8? (캐스팅됨)
        unpack_and_advance("h", "bms.soc_limit_start", scale=1000)
        unpack_and_advance("h", "bms.soc_limit_end", scale=1000)
        unpack_and_advance("h", "bms.vmin_limit_start", scale=1000)
        unpack_and_advance("h", "bms.vmin_limit_end", scale=1000)
        unpack_and_advance("h", "bms.vmax_limit_start", scale=1000)
        unpack_and_advance("h", "bms.vmax_limit_end", scale=1000)
        unpack_and_advance("B", "bms.fwd_can_mode")
        # --- 여기까지가 BMS 끝 ---

        print(f"\nBMS 필드까지 파싱 완료.")
        print(f"현재 오프셋: {offset}")
        remaining_bytes = buffer_len - offset
        print(f"남은 바이트 수: {remaining_bytes}")

        # --- 마지막 CRC 파싱 시도 ---
        if remaining_bytes >= 2:  # CRC 포함하여 딱 맞거나 더 클 경우
            # CRC는 보통 데이터 무결성 검사용이므로, 마지막 2바이트로 가정
            crc_offset = buffer_len - 2
            try:
                crc_fmt = ">H"
                crc_value = struct.unpack_from(crc_fmt, payload_buffer, crc_offset)[0]
                parsed_config["crc"] = crc_value
                # CRC는 마지막이므로 오프셋을 buffer_len으로 설정하여 길이 검증
                offset = buffer_len
                print(
                    f"마지막 2바이트를 CRC로 파싱 성공 (offset={crc_offset}): {crc_value}"
                )
            except (struct.error, IndexError) as e:
                print(f"\n오류: 마지막 2바이트 CRC 파싱 실패. {e}")
        # ... (이전 CRC 관련 경고 메시지 동일) ...
        elif remaining_bytes > 0:
            print(
                f"\n경고: CRC를 파싱하기에 남은 바이트({remaining_bytes})가 부족합니다."
            )
            print(
                f"  남은 데이터 (Hex): {' '.join(f'{b:02x}' for b in payload_buffer[offset:])}"
            )

        print(f"\n파싱 완료! 최종 오프셋: {offset} / 페이로드 길이: {buffer_len}")
        if offset != buffer_len:
            print("경고: 파싱 완료 후 오프셋과 페이로드 길이가 일치하지 않습니다!")

        return parsed_config

    except (struct.error, IndexError) as e:
        print("\n파싱 중단됨.")
        return parsed_config
    except Exception as e:
        print(f"\n예상치 못한 파싱 오류 발생: {e}")
        return parsed_config


# -----------------------------------


# --- 메인 실행 부분 ---
if __name__ == "__main__":
    vesc_serial = None
    try:
        vesc_serial = connect_to_vesc(SERIAL_PORT, BAUD_RATE, TIMEOUT)

        if vesc_serial:
            print("\n--- COMM_GET_MCCONF 요청 및 파싱 시도 (serialize 함수 기반) ---")

            # 1. 요청 전 버퍼 비우기
            print("요청 전 입력 버퍼 정리...")
            clear_input_buffer(vesc_serial, wait_time=0.1)

            # 2. 요청 생성 및 전송 (커스텀 ID 14 사용)
            request_packet = pyvesc.encode_request(GetMcConfCustom)
            print("COMM_GET_MCCONF (ID 14) 요청 전송 중...")
            vesc_serial.write(request_packet)
            print("요청 전송 완료.")

            # 3. VESC 응답 대기
            response_wait_time = 1.0
            print(f"{response_wait_time}초 동안 응답 대기...")
            time.sleep(response_wait_time)

            # 4. 응답 읽기 시도
            read_size = 4096
            print(f"최대 {read_size} 바이트 응답 읽기 시도...")
            mcconf_buffer = vesc_serial.read(read_size)

            if mcconf_buffer:
                print(f"\n수신 성공: {len(mcconf_buffer)} 바이트의 원시 데이터 수신.")
                print(f"수신된 데이터 (앞 100바이트): {mcconf_buffer[:100]}...")

                # 5. 파싱 수행
                header_size = 4  # 시작(1) + 길이(2) + ID(1)
                if len(mcconf_buffer) > header_size:
                    response_id = mcconf_buffer[3]
                    print(f"  응답 패킷 ID: {response_id} (0x{response_id:02x})")

                    payload = mcconf_buffer[header_size:]

                    # *** 최종 파싱 함수 호출 ***
                    parsed_motor_config = parse_mc_conf_serialized(payload)

                    if parsed_motor_config:
                        print("\n--- 파싱된 모터 구성 결과 ---")
                        pprint.pprint(parsed_motor_config)
                        print("---------------------------------")

                        # *** 값 비교를 위한 추가 출력 ***
                        print("\n--- VESC Tool 값과 비교 ---")
                        vesc_tool_values = {
                            "l_current_max": 48.94,
                            "comm_mode": 0,
                            "foc_cc_decoupling": 0,
                            # 'foc_control_sample_mode': 0, # 이 필드는 serialize 함수에 없음
                            "foc_current_filter_const": 0.1,
                            "foc_current_ki": 33.40,
                            "foc_current_kp": 0.0532,
                        }
                        comparison_results = {}
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
                            comparison_results[key] = status

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
                        print("\n오류: 모터 구성 데이터 파싱에 실패했습니다.")
                else:
                    print("\n오류: 수신된 데이터가 너무 짧아 파싱할 수 없습니다.")

            else:
                print(
                    "\n오류: COMM_GET_MCCONF에 대한 응답을 받지 못했습니다 (타임아웃)."
                )

        else:
            print("\nVESC 연결 실패.")

    except KeyboardInterrupt:
        print("\n사용자에 의해 프로그램 중단됨.")
    finally:
        close_serial_port(vesc_serial)
        print("프로그램 종료.")
