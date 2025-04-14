# vesc_parser.py
import struct
import pprint
import traceback  # 상세 오류 출력을 위해 임포트


# --- mc_configuration 파싱 함수 (최종 디버깅 버전) ---
def parse_mc_conf_serialized(payload_buffer):
    parsed_config = {}
    offset = 0
    buffer_len = len(payload_buffer)
    print(
        f"mc_configuration (Serialized) 파싱 시작 (페이로드 크기: {buffer_len} 바이트)..."
    )
    print(
        f"페이로드 시작 (Hex): {' '.join(f'{b:02x}' for b in payload_buffer[:32])}..."
    )

    # 내부 헬퍼 함수 (디버깅 로그 포함된 버전 유지)
    def unpack_and_advance(fmt, field_name, scale=None):
        nonlocal offset
        print(
            f"  Attempting to parse '{field_name}' at offset {offset} with format '{'>' + fmt}'..."
        )  # 시도 로깅
        try:
            full_fmt = ">" + fmt
            size = struct.calcsize(full_fmt)
            if offset + size > buffer_len:
                if field_name != "crc":
                    print(f"\n오류: 필드 '{field_name}' 파싱 중 버퍼 길이 초과...")
                    raise IndexError(
                        f"Buffer length exceeded parsing field '{field_name}'"
                    )
                else:
                    return None
            value = struct.unpack_from(full_fmt, payload_buffer, offset)[0]
            if scale is not None and scale != 0:
                value = float(value) / scale
            parsed_config[field_name] = value
            value_repr = (
                f"{value:.4f}"
                if isinstance(value, float) and scale is not None
                else str(value)
            )
            print(
                f"    OK: Parsed '{field_name}' = {value_repr} (offset now {offset + size}, size={size}, fmt='{full_fmt}')"
            )  # 성공 로그
            offset += size
            return value
        except (struct.error, IndexError) as e:
            print(f"\n오류: 필드 '{field_name}' 파싱 중 오류 발생 (struct/Index)!")
            print(f"  Offset: {offset}, Format tried: '{full_fmt}', Error: {e}")
            raise  # 오류 다시 발생
        except Exception as e:
            print(f"\n오류: 필드 '{field_name}' 파싱 중 예상치 못한 오류!")
            print(f"  Offset: {offset}, Format tried: '{full_fmt}', Error: {e}")
            raise  # 오류 다시 발생

    # *** 메인 파싱 로직을 포괄적인 try...except로 감싸기 ***
    try:
        # --- confgenerator_serialize_mcconf 순서대로 파싱 ---
        # 1. Signature
        unpack_and_advance("I", "MCCONF_SIGNATURE")
        # 2. Enum/bool
        unpack_and_advance("B", "pwm_mode")
        unpack_and_advance("B", "comm_mode")
        unpack_and_advance("B", "motor_type")  # 여기서 오류 발생 가능성?
        unpack_and_advance("B", "sensor_mode")
        # 3. Limits
        unpack_and_advance("f", "l_current_max")  # 또는 여기서?
        unpack_and_advance("f", "l_current_min")
        # ... (이전 코드에서 완성된 모든 필드 파싱 로직) ...
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
        unpack_and_advance("?", "l_slow_abs_current")
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
        unpack_and_advance("f", "sl_min_erpm")
        unpack_and_advance("f", "sl_min_erpm_cycle_int_limit")
        unpack_and_advance("f", "sl_max_fullbreak_current_dir_change")
        unpack_and_advance("h", "sl_cycle_int_limit", scale=10)
        unpack_and_advance("h", "sl_phase_advance_at_br", scale=10000)
        unpack_and_advance("f", "sl_cycle_int_rpm_br")
        unpack_and_advance("f", "sl_bemf_coupling_k")
        fmt = ">8b"
        size = struct.calcsize(fmt)
        print(
            f"  Attempting to parse 'hall_table' at offset {offset} with format '{fmt}'..."
        )
        if offset + size > buffer_len:
            raise IndexError("Buffer length exceeded for hall_table")
        hall_table = struct.unpack_from(fmt, payload_buffer, offset)
        parsed_config["hall_table"] = list(hall_table)
        print(
            f"    OK: Parsed 'hall_table' = {list(hall_table)} (offset now {offset + size}, size={size}, fmt='{fmt}')"
        )
        offset += size
        unpack_and_advance("f", "hall_sl_erpm")
        unpack_and_advance("f", "foc_current_kp")
        unpack_and_advance("f", "foc_current_ki")
        unpack_and_advance("f", "foc_f_zv")
        unpack_and_advance("f", "foc_dt_us")
        unpack_and_advance("?", "foc_encoder_inverted")
        unpack_and_advance("f", "foc_encoder_offset")
        unpack_and_advance("f", "foc_encoder_ratio")
        unpack_and_advance("B", "foc_sensor_mode")
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
        fmt = ">8B"
        size = struct.calcsize(fmt)
        print(
            f"  Attempting to parse 'foc_hall_table' at offset {offset} with format '{fmt}'..."
        )
        if offset + size > buffer_len:
            raise IndexError("Buffer length exceeded for foc_hall_table")
        foc_hall_table = struct.unpack_from(fmt, payload_buffer, offset)
        parsed_config["foc_hall_table"] = list(foc_hall_table)
        print(
            f"    OK: Parsed 'foc_hall_table' = {list(foc_hall_table)} (offset now {offset + size}, size={size}, fmt='{fmt}')"
        )
        offset += size
        unpack_and_advance("f", "foc_hall_interp_erpm")
        unpack_and_advance("f", "foc_sl_erpm_start")
        unpack_and_advance("f", "foc_sl_erpm")
        unpack_and_advance("B", "foc_control_sample_mode")
        unpack_and_advance("B", "foc_current_sample_mode")
        unpack_and_advance("B", "foc_sat_comp_mode")
        unpack_and_advance("h", "foc_sat_comp", scale=1000)
        unpack_and_advance("?", "foc_temp_comp")
        unpack_and_advance("h", "foc_temp_comp_base_temp", scale=100)
        unpack_and_advance("h", "foc_current_filter_const", scale=10000)
        unpack_and_advance("B", "foc_cc_decoupling")
        unpack_and_advance("B", "foc_observer_type")
        unpack_and_advance("h", "foc_hfi_voltage_start", scale=10)
        unpack_and_advance("h", "foc_hfi_voltage_run", scale=10)
        unpack_and_advance("h", "foc_hfi_voltage_max", scale=10)
        unpack_and_advance("h", "foc_hfi_gain", scale=1000)
        unpack_and_advance("h", "foc_hfi_max_err", scale=1000)
        unpack_and_advance("h", "foc_hfi_hyst", scale=100)
        unpack_and_advance("f", "foc_sl_erpm_hfi")
        unpack_and_advance("H", "foc_hfi_start_samples")
        unpack_and_advance("f", "foc_hfi_obs_ovr_sec")
        unpack_and_advance("B", "foc_hfi_samples")
        unpack_and_advance("?", "foc_offsets_cal_on_boot")
        fmt = ">3f"
        size = struct.calcsize(fmt)
        print(
            f"  Attempting to parse 'foc_offsets_current' at offset {offset} with format '{fmt}'..."
        )
        if offset + size > buffer_len:
            raise IndexError("Buffer length exceeded for foc_offsets_current")
        foc_offsets_current = struct.unpack_from(fmt, payload_buffer, offset)
        parsed_config["foc_offsets_current"] = list(foc_offsets_current)
        print(
            f"    OK: Parsed 'foc_offsets_current' = {list(foc_offsets_current)} (offset now {offset + size}, size={size}, fmt='{fmt}')"
        )
        offset += size
        fmt = ">3h"
        size = struct.calcsize(fmt)
        print(
            f"  Attempting to parse 'foc_offsets_voltage' at offset {offset} with format '{fmt}'..."
        )
        if offset + size > buffer_len:
            raise IndexError("Buffer length exceeded for foc_offsets_voltage")
        foc_offsets_voltage_raw = struct.unpack_from(fmt, payload_buffer, offset)
        parsed_config["foc_offsets_voltage"] = [
            v / 10000.0 for v in foc_offsets_voltage_raw
        ]
        print(
            f"    OK: Parsed 'foc_offsets_voltage' = {[f'{v:.4f}' for v in parsed_config['foc_offsets_voltage']]} (offset now {offset + size}, size={size}, fmt='{fmt}')"
        )
        offset += size
        fmt = ">3h"
        size = struct.calcsize(fmt)
        print(
            f"  Attempting to parse 'foc_offsets_voltage_undriven' at offset {offset} with format '{fmt}'..."
        )
        if offset + size > buffer_len:
            raise IndexError("Buffer length exceeded for foc_offsets_voltage_undriven")
        foc_offsets_voltage_undriven_raw = struct.unpack_from(
            fmt, payload_buffer, offset
        )
        parsed_config["foc_offsets_voltage_undriven"] = [
            v / 10000.0 for v in foc_offsets_voltage_undriven_raw
        ]
        print(
            f"    OK: Parsed 'foc_offsets_voltage_undriven' = {[f'{v:.4f}' for v in parsed_config['foc_offsets_voltage_undriven']]} (offset now {offset + size}, size={size}, fmt='{fmt}')"
        )
        offset += size
        unpack_and_advance("?", "foc_phase_filter_enable")
        unpack_and_advance("?", "foc_phase_filter_disable_fault")
        unpack_and_advance("f", "foc_phase_filter_max_erpm")
        unpack_and_advance("B", "foc_mtpa_mode")
        unpack_and_advance("f", "foc_fw_current_max")
        unpack_and_advance("h", "foc_fw_duty_start", scale=10000)
        unpack_and_advance("h", "foc_fw_ramp_time", scale=1000)
        unpack_and_advance("h", "foc_fw_q_current_factor", scale=10000)
        unpack_and_advance("B", "foc_speed_soure")  # Typo
        unpack_and_advance("?", "foc_short_ls_on_zero_duty")
        unpack_and_advance("B", "sp_pid_loop_rate")
        unpack_and_advance("f", "s_pid_kp")
        unpack_and_advance("f", "s_pid_ki")
        unpack_and_advance("f", "s_pid_kd")
        unpack_and_advance("h", "s_pid_kd_filter", scale=10000)
        unpack_and_advance("f", "s_pid_min_erpm")
        unpack_and_advance("?", "s_pid_allow_braking")
        unpack_and_advance("f", "s_pid_ramp_erpms_s")
        unpack_and_advance("B", "s_pid_speed_source")
        unpack_and_advance("f", "p_pid_kp")
        unpack_and_advance("f", "p_pid_ki")
        unpack_and_advance("f", "p_pid_kd")
        unpack_and_advance("f", "p_pid_kd_proc")
        unpack_and_advance("h", "p_pid_kd_filter", scale=10000)
        unpack_and_advance("f", "p_pid_ang_div")
        unpack_and_advance("h", "p_pid_gain_dec_angle", scale=10)
        unpack_and_advance("f", "p_pid_offset")
        unpack_and_advance("h", "cc_startup_boost_duty", scale=10000)
        unpack_and_advance("f", "cc_min_current")
        unpack_and_advance("f", "cc_gain")
        unpack_and_advance("h", "cc_ramp_step_max", scale=10000)
        unpack_and_advance("i", "m_fault_stop_time_ms")
        unpack_and_advance("h", "m_duty_ramp_step", scale=10000)
        unpack_and_advance("f", "m_current_backoff_gain")
        unpack_and_advance("I", "m_encoder_counts")
        unpack_and_advance("h", "m_encoder_sin_amp", scale=1000)
        unpack_and_advance("h", "m_encoder_cos_amp", scale=1000)
        unpack_and_advance("h", "m_encoder_sin_offset", scale=1000)
        unpack_and_advance("h", "m_encoder_cos_offset", scale=1000)
        unpack_and_advance("h", "m_encoder_sincos_filter_constant", scale=1000)
        unpack_and_advance("h", "m_encoder_sincos_phase_correction", scale=1000)
        unpack_and_advance("B", "m_sensor_port_mode")
        unpack_and_advance("?", "m_invert_direction")
        unpack_and_advance("B", "m_drv8301_oc_mode")
        unpack_and_advance("B", "m_drv8301_oc_adj")  # uint8 cast
        unpack_and_advance("f", "m_bldc_f_sw_min")
        unpack_and_advance("f", "m_bldc_f_sw_max")
        unpack_and_advance("f", "m_dc_f_sw")
        unpack_and_advance("f", "m_ntc_motor_beta")
        unpack_and_advance("B", "m_out_aux_mode")
        unpack_and_advance("B", "m_motor_temp_sens_type")
        unpack_and_advance("f", "m_ptc_motor_coeff")
        unpack_and_advance("h", "m_ntcx_ptcx_res", scale=0.1)
        unpack_and_advance("h", "m_ntcx_ptcx_temp_base", scale=10)
        unpack_and_advance("B", "m_hall_extra_samples")  # uint8 cast
        unpack_and_advance("B", "m_batt_filter_const")  # uint8 cast
        unpack_and_advance("B", "si_motor_poles")
        unpack_and_advance("f", "si_gear_ratio")
        unpack_and_advance("f", "si_wheel_diameter")
        unpack_and_advance("B", "si_battery_type")
        unpack_and_advance("B", "si_battery_cells")  # uint8 cast
        unpack_and_advance("f", "si_battery_ah")
        unpack_and_advance("f", "si_motor_nl_current")
        # BMS Config
        unpack_and_advance("B", "bms.type")
        unpack_and_advance("B", "bms.limit_mode")
        unpack_and_advance("B", "bms.t_limit_start")  # uint8 cast
        unpack_and_advance("B", "bms.t_limit_end")  # uint8 cast
        unpack_and_advance("h", "bms.soc_limit_start", scale=1000)
        unpack_and_advance("h", "bms.soc_limit_end", scale=1000)
        unpack_and_advance("h", "bms.vmin_limit_start", scale=1000)
        unpack_and_advance("h", "bms.vmin_limit_end", scale=1000)
        unpack_and_advance("h", "bms.vmax_limit_start", scale=1000)
        unpack_and_advance("h", "bms.vmax_limit_end", scale=1000)
        unpack_and_advance("B", "bms.fwd_can_mode")

        # --- BMS 끝, CRC 파싱 시도 ---
        print(f"\nBMS 필드까지 파싱 완료. 현재 오프셋: {offset}")
        remaining_bytes = buffer_len - offset
        print(f"남은 바이트 수: {remaining_bytes}")

        if remaining_bytes >= 2:
            crc_offset = buffer_len - 2
            try:
                crc_fmt = ">H"
                crc_value = struct.unpack_from(crc_fmt, payload_buffer, crc_offset)[0]
                parsed_config["crc"] = crc_value
                offset = buffer_len
                print(
                    f"마지막 2바이트를 CRC로 파싱 성공 (offset={crc_offset}): {crc_value}"
                )
            except (struct.error, IndexError) as e:
                print(f"\n오류: 마지막 2바이트 CRC 파싱 실패. {e}")
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

    # *** 포괄적인 예외 처리 추가 ***
    except Exception as e:
        print(f"\n!!! 파싱 중 예상치 못한 오류 발생 !!!")
        # traceback 모듈을 임포트해서 더 자세한 정보 얻기
        # (vesc_parser.py 상단에 import traceback 추가 필요)
        print(traceback.format_exc())  # 스택 트레이스 출력
        print(f"  오류 발생 시점 추정 offset: {offset}")
        # 오류 발생 지점 근처 바이트 출력
        start_byte = max(0, offset - 8)
        end_byte = min(buffer_len, offset + 8)
        print(
            f"  Bytes around offset {offset} (Hex): {' '.join(f'{b:02x}' for b in payload_buffer[start_byte:end_byte])}"
        )
        return parsed_config  # 부분 결과 반환
