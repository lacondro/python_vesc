# vesc_parser.py
import struct
import math
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
            # f"  Attempting to parse '{field_name}' at offset {offset} with format '{'>' + fmt}'..."
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
            # f"  Attempting to parse 'hall_table' at offset {offset} with format '{fmt}'..."
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
            # f"  Attempting to parse 'foc_hall_table' at offset {offset} with format '{fmt}'..."
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
            # f"  Attempting to parse 'foc_offsets_current' at offset {offset} with format '{fmt}'..."
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
            # f"  Attempting to parse 'foc_offsets_voltage' at offset {offset} with format '{fmt}'..."
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
            # f"  Attempting to parse 'foc_offsets_voltage_undriven' at offset {offset} with format '{fmt}'..."
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


DEBUG_PACKER = False  # True로 설정하면 상세 로그 출력


def pack_mc_conf_serialized(config_dict):
    """
    [DEBUG VERSION] mc_configuration 딕셔너리를 VESC COMM_SET_MCCONF 명령을 위한
    바이트 페이로드로 직렬화(packing)합니다.
    입력 딕셔너리에 포함된 'MCCONF_SIGNATURE' 값을 사용하며, 각 단계 로그를 출력합니다.
    """
    packed_data = bytearray()
    if DEBUG_PACKER:
        print("\n--- pack_mc_conf_serialized: Packing 시작 ---")

    # === 단계 1: Signature 처리 ===
    if DEBUG_PACKER:
        print("[DEBUG] Signature 처리 시도...")
    try:
        # pop 메서드는 값을 반환하고 딕셔너리에서 해당 키를 제거합니다.
        signature_to_pack = config_dict.pop("MCCONF_SIGNATURE")
        if DEBUG_PACKER:
            print(
                f"[DEBUG]   Signature 값 0x{signature_to_pack:X} 추출 완료 (dict에서 제거됨)"
            )
    except KeyError:
        print(
            "!!![PACKER ERROR] 입력 config_dict에 'MCCONF_SIGNATURE' 키가 없습니다. GET 요청 실패 또는 파서 오류일 수 있습니다."
        )
        raise  # 필수 값이므로 오류 발생시킴

    try:
        packed_sig = struct.pack(">I", signature_to_pack)
        packed_data.extend(packed_sig)
        if DEBUG_PACKER:
            print(f"[DEBUG]   Signature 패킹 성공. 현재 크기: {len(packed_data)} bytes")
    except Exception as e:
        print(
            f"!!![PACKER ERROR] Signature (0x{signature_to_pack:X}) 패킹 중 오류: {e}"
        )
        raise

    # === 내부 헬퍼 함수 (디버깅 강화) ===
    def _pack_value(fmt, field_name, scale=None, is_array=False, array_len=0):
        nonlocal packed_data
        start_len = len(packed_data)  # 현재 길이 저장
        if DEBUG_PACKER:
            print(
                f"\n[DEBUG] 필드 '{field_name}' (fmt='{fmt}', scale={scale}) 처리 시도..."
            )

        if field_name not in config_dict:
            # CRC는 frame() 함수가 처리하므로 packer에서는 무시
            if field_name == "crc":
                if DEBUG_PACKER:
                    print(
                        f"[DEBUG]   '{field_name}' 필드는 건너<0xEB><0x9A><0x84>니다 (외부 처리)."
                    )
                return
            print(
                f"!!![PACKER ERROR] 필수 필드 '{field_name}'가 config_dict에 없습니다!"
            )
            # 디버깅 위해 현재 dict 내용 일부 출력
            # print("[DEBUG] Current keys:", list(config_dict.keys())[:20], "...")
            raise KeyError(f"필수 필드 '{field_name}' 누락")

        value = config_dict[field_name]
        original_value_repr = repr(value)
        if DEBUG_PACKER:
            print(f"[DEBUG]   원본 값: {original_value_repr}")

        try:
            packed_value_bytes = b""
            processed_value_repr = original_value_repr  # 기본값

            if is_array:  # --- 배열 처리 ---
                if not isinstance(value, (list, tuple)):
                    raise TypeError(
                        f"'{field_name}'는 리스트/튜플이어야 함 (현재: {type(value)})"
                    )
                if len(value) != array_len:
                    raise ValueError(
                        f"'{field_name}' 길이는 {array_len}여야 함 (현재: {len(value)})"
                    )

                full_fmt = f">{array_len}{fmt}"
                values_to_pack = []

                # 스케일링 필요한 배열 (주로 float16) 처리
                if scale is not None and scale != 0 and fmt == "h":
                    scaled_list = []
                    for i, v_item in enumerate(value):
                        try:
                            scaled = int(round(float(v_item) * scale))
                            # int16 범위 체크 및 클램핑
                            if not (-32768 <= scaled <= 32767):
                                if DEBUG_PACKER:
                                    print(
                                        f"[DEBUG]   경고: '{field_name}' 배열 요소 #{i} ({v_item}) 스케일링 값 {scaled}이 int16 범위를 벗어남. 클램핑 적용."
                                    )
                                scaled = max(-32768, min(32767, scaled))
                            scaled_list.append(scaled)
                        except (ValueError, TypeError) as ve:
                            raise ValueError(
                                f"'{field_name}' 배열 요소 #{i} ({v_item}) 스케일링/변환 오류: {ve}"
                            )
                    values_to_pack = scaled_list
                    processed_value_repr = repr(scaled_list)
                else:  # 다른 타입 배열 (int8, uint8, float32 등)
                    # 타입 변환은 struct.pack에 맡김 (필요시 여기서 강제 변환 추가 가능)
                    values_to_pack = list(
                        value
                    )  # struct.pack은 가변 인자를 받으므로 리스트로 변환

                packed_value_bytes = struct.pack(full_fmt, *values_to_pack)

            else:  # --- 단일 값 처리 ---
                processed_value = value
                full_fmt = f">{fmt}"

                # 타입별 변환, 스케일링, 범위 체크
                if fmt == "f":  # float32
                    try:
                        processed_value = float(value)
                    except (ValueError, TypeError) as e:
                        raise TypeError(f"float 변환 불가: {value} ({e})")
                    if math.isnan(processed_value) or math.isinf(processed_value):
                        if DEBUG_PACKER:
                            print(
                                f"[DEBUG]   경고: '{field_name}' 값이 NaN/Inf. 0.0으로 대체."
                            )
                        processed_value = 0.0
                elif fmt == "h":  # float16 (-> int16)
                    if scale is None or scale == 0:
                        raise ValueError(
                            "float16 ('h') 포맷에는 유효한 scale 값이 필요합니다."
                        )
                    try:
                        processed_value = int(round(float(value) * scale))
                    except (ValueError, TypeError) as e:
                        raise TypeError(f"float/int 변환 불가: {value} ({e})")
                    if not (-32768 <= processed_value <= 32767):
                        if DEBUG_PACKER:
                            print(
                                f"[DEBUG]   경고: '{field_name}' 스케일링 값 {processed_value}가 int16 범위를 벗어남. 클램핑 적용."
                            )
                        processed_value = max(-32768, min(32767, processed_value))
                    processed_value_repr = str(processed_value)  # 로그용
                elif fmt == "i":  # int32
                    try:
                        processed_value = int(value)
                    except (ValueError, TypeError) as e:
                        raise TypeError(f"int 변환 불가: {value} ({e})")
                    # 범위 체크는 struct.pack 에서 처리 (필요시 추가)
                elif fmt == "I":  # uint32
                    try:
                        processed_value = int(value)
                    except (ValueError, TypeError) as e:
                        raise TypeError(f"int 변환 불가: {value} ({e})")
                    if not (0 <= processed_value <= 4294967295):
                        if DEBUG_PACKER:
                            print(
                                f"[DEBUG]   경고: '{field_name}' 값 {processed_value}가 uint32 범위를 벗어남. 클램핑 적용."
                            )
                        processed_value = max(0, min(4294967295, processed_value))
                elif fmt == "b":  # int8
                    try:
                        processed_value = int(value)
                    except (ValueError, TypeError) as e:
                        raise TypeError(f"int 변환 불가: {value} ({e})")
                    if not (-128 <= processed_value <= 127):
                        if DEBUG_PACKER:
                            print(
                                f"[DEBUG]   경고: '{field_name}' 값 {processed_value}가 int8 범위를 벗어남. 클램핑 적용."
                            )
                        processed_value = max(-128, min(127, processed_value))
                elif fmt == "B":  # uint8
                    try:
                        # float -> int 변환 지원 (예: 온도)
                        if isinstance(value, float):
                            processed_value = int(round(value))
                        else:
                            processed_value = int(value)
                    except (ValueError, TypeError) as e:
                        raise TypeError(f"int 변환 불가: {value} ({e})")
                    if not (0 <= processed_value <= 255):
                        if DEBUG_PACKER:
                            print(
                                f"[DEBUG]   경고: '{field_name}' 값 {processed_value}가 uint8 범위를 벗어남. 클램핑 적용."
                            )
                        processed_value = max(0, min(255, processed_value))
                    processed_value_repr = str(processed_value)  # 로그용
                elif fmt == "H":  # uint16
                    try:
                        processed_value = int(value)
                    except (ValueError, TypeError) as e:
                        raise TypeError(f"int 변환 불가: {value} ({e})")
                    if not (0 <= processed_value <= 65535):
                        if DEBUG_PACKER:
                            print(
                                f"[DEBUG]   경고: '{field_name}' 값 {processed_value}가 uint16 범위를 벗어남. 클램핑 적용."
                            )
                        processed_value = max(0, min(65535, processed_value))
                elif fmt == "?":  # bool (-> int 0/1)
                    processed_value = int(bool(value))
                    processed_value_repr = str(processed_value)  # 로그용
                else:
                    raise ValueError(f"지원하지 않는 포맷 문자: '{fmt}'")

                packed_value_bytes = struct.pack(full_fmt, processed_value)

            # 성공 로그
            packed_data.extend(packed_value_bytes)
            if DEBUG_PACKER:
                packed_hex = packed_value_bytes.hex(" ")
                print(
                    f"[DEBUG]   OK: Packed '{field_name}'. Original: {original_value_repr}, Processed: {processed_value_repr}, Hex: {packed_hex}, Size: {len(packed_value_bytes)} bytes."
                )
                print(f"[DEBUG]   ===> 현재 총 크기: {len(packed_data)} bytes")

        except KeyError as e:
            raise  # 위에서 이미 잡았지만 명시적으로 다시 발생
        except (struct.error, TypeError, ValueError) as e:
            print(f"\n!!![PACKER ERROR] 필드 '{field_name}' 패킹 중 오류 발생 !!!")
            print(
                f"  원본 값: {original_value_repr}, 포맷: '{fmt}', 스케일: {scale}, 에러: {e}"
            )
            # 문제가 된 값과 주변 데이터 출력 (디버깅에 도움)
            # print(f"  Current packed_data (last 10 bytes): {packed_data[-10:].hex(' ')}")
            raise  # 오류 발생 시 중단

    # --- confgenerator_deserialize_mcconf 의 역순으로 나머지 필드 패킹 ---
    # 각 섹션 시작 시 로그 추가
    if DEBUG_PACKER:
        print("\n--- [DEBUG] Packing Enums/Bools ---")
    _pack_value("B", "pwm_mode")  # uint8_t
    _pack_value("B", "comm_mode")  # uint8_t
    _pack_value("B", "motor_type")  # uint8_t
    _pack_value("B", "sensor_mode")  # uint8_t

    if DEBUG_PACKER:
        print("\n--- [DEBUG] Packing Limits ---")
    _pack_value("f", "l_current_max")  # float32
    _pack_value("f", "l_current_min")  # float32
    _pack_value("f", "l_in_current_max")  # float32
    _pack_value("f", "l_in_current_min")  # float32
    _pack_value("h", "l_in_current_map_start", scale=10000)  # float16
    _pack_value("h", "l_in_current_map_filter", scale=10000)  # float16
    _pack_value("f", "l_abs_current_max")  # float32
    _pack_value("f", "l_min_erpm")  # float32
    _pack_value("f", "l_max_erpm")  # float32
    _pack_value("h", "l_erpm_start", scale=10000)  # float16
    _pack_value("f", "l_max_erpm_fbrake")  # float32
    _pack_value("f", "l_max_erpm_fbrake_cc")  # float32
    _pack_value("h", "l_min_vin", scale=10)  # float16
    _pack_value("h", "l_max_vin", scale=10)  # float16
    _pack_value("h", "l_battery_cut_start", scale=10)  # float16
    _pack_value("h", "l_battery_cut_end", scale=10)  # float16
    _pack_value("h", "l_battery_regen_cut_start", scale=10)  # float16
    _pack_value("h", "l_battery_regen_cut_end", scale=10)  # float16
    _pack_value("?", "l_slow_abs_current")  # bool (-> uint8_t)
    _pack_value("B", "l_temp_fet_start")  # uint8_t (C에서는 float 캐스팅 가능성 있음)
    _pack_value("B", "l_temp_fet_end")  # uint8_t
    _pack_value("B", "l_temp_motor_start")  # uint8_t
    _pack_value("B", "l_temp_motor_end")  # uint8_t
    _pack_value("h", "l_temp_accel_dec", scale=10000)  # float16
    _pack_value("h", "l_min_duty", scale=10000)  # float16
    _pack_value("h", "l_max_duty", scale=10000)  # float16
    _pack_value("f", "l_watt_max")  # float32
    _pack_value("f", "l_watt_min")  # float32
    _pack_value("h", "l_current_max_scale", scale=10000)  # float16
    _pack_value("h", "l_current_min_scale", scale=10000)  # float16
    _pack_value("h", "l_duty_start", scale=10000)  # float16

    if DEBUG_PACKER:
        print("\n--- [DEBUG] Packing Sensorless ---")
    _pack_value("f", "sl_min_erpm")  # float32
    _pack_value("f", "sl_min_erpm_cycle_int_limit")  # float32
    _pack_value("f", "sl_max_fullbreak_current_dir_change")  # float32
    _pack_value("h", "sl_cycle_int_limit", scale=10)  # float16
    _pack_value("h", "sl_phase_advance_at_br", scale=10000)  # float16
    _pack_value("f", "sl_cycle_int_rpm_br")  # float32
    _pack_value("f", "sl_bemf_coupling_k")  # float32
    _pack_value("b", "hall_table", is_array=True, array_len=8)  # int8[8]
    _pack_value("f", "hall_sl_erpm")  # float32

    if DEBUG_PACKER:
        print("\n--- [DEBUG] Packing FOC ---")
    _pack_value("f", "foc_current_kp")  # float32
    _pack_value("f", "foc_current_ki")  # float32
    _pack_value("f", "foc_f_zv")  # float32
    _pack_value("f", "foc_dt_us")  # float32
    _pack_value("?", "foc_encoder_inverted")  # bool (-> uint8_t)
    _pack_value("f", "foc_encoder_offset")  # float32
    _pack_value("f", "foc_encoder_ratio")  # float32
    _pack_value("B", "foc_sensor_mode")  # uint8_t
    _pack_value("f", "foc_pll_kp")  # float32
    _pack_value("f", "foc_pll_ki")  # float32
    _pack_value("f", "foc_motor_l")  # float32
    _pack_value("f", "foc_motor_ld_lq_diff")  # float32
    _pack_value("f", "foc_motor_r")  # float32
    _pack_value("f", "foc_motor_flux_linkage")  # float32
    _pack_value("f", "foc_observer_gain")  # float32
    _pack_value("f", "foc_observer_gain_slow")  # float32
    _pack_value("h", "foc_observer_offset", scale=1000)  # float16
    _pack_value("f", "foc_duty_dowmramp_kp")  # float32
    _pack_value("f", "foc_duty_dowmramp_ki")  # float32
    _pack_value("h", "foc_start_curr_dec", scale=10000)  # float16
    _pack_value("f", "foc_start_curr_dec_rpm")  # float32
    _pack_value("f", "foc_openloop_rpm")  # float32
    _pack_value("h", "foc_openloop_rpm_low", scale=1000)  # float16
    _pack_value("h", "foc_d_gain_scale_start", scale=1000)  # float16
    _pack_value("h", "foc_d_gain_scale_max_mod", scale=1000)  # float16
    _pack_value("h", "foc_sl_openloop_hyst", scale=100)  # float16
    _pack_value("h", "foc_sl_openloop_time_lock", scale=100)  # float16
    _pack_value("h", "foc_sl_openloop_time_ramp", scale=100)  # float16
    _pack_value("h", "foc_sl_openloop_time", scale=100)  # float16
    _pack_value("h", "foc_sl_openloop_boost_q", scale=100)  # float16
    _pack_value("h", "foc_sl_openloop_max_q", scale=100)  # float16
    _pack_value("B", "foc_hall_table", is_array=True, array_len=8)  # uint8[8]
    _pack_value("f", "foc_hall_interp_erpm")  # float32
    _pack_value("f", "foc_sl_erpm_start")  # float32
    _pack_value("f", "foc_sl_erpm")  # float32
    _pack_value("B", "foc_control_sample_mode")  # uint8_t
    _pack_value("B", "foc_current_sample_mode")  # uint8_t
    _pack_value("B", "foc_sat_comp_mode")  # uint8_t
    _pack_value("h", "foc_sat_comp", scale=1000)  # float16
    _pack_value("?", "foc_temp_comp")  # bool (-> uint8_t)
    _pack_value("h", "foc_temp_comp_base_temp", scale=100)  # float16
    _pack_value("h", "foc_current_filter_const", scale=10000)  # float16
    _pack_value("B", "foc_cc_decoupling")  # uint8_t
    _pack_value("B", "foc_observer_type")  # uint8_t
    _pack_value("h", "foc_hfi_voltage_start", scale=10)  # float16
    _pack_value("h", "foc_hfi_voltage_run", scale=10)  # float16
    _pack_value("h", "foc_hfi_voltage_max", scale=10)  # float16
    _pack_value("h", "foc_hfi_gain", scale=1000)  # float16
    _pack_value("h", "foc_hfi_max_err", scale=1000)  # float16
    _pack_value("h", "foc_hfi_hyst", scale=100)  # float16
    _pack_value("f", "foc_sl_erpm_hfi")  # float32
    _pack_value("H", "foc_hfi_start_samples")  # uint16
    _pack_value("f", "foc_hfi_obs_ovr_sec")  # float32
    _pack_value("B", "foc_hfi_samples")  # uint8_t
    _pack_value("?", "foc_offsets_cal_on_boot")  # bool (-> uint8_t)
    _pack_value("f", "foc_offsets_current", is_array=True, array_len=3)  # float32[3]
    _pack_value(
        "h", "foc_offsets_voltage", scale=10000, is_array=True, array_len=3
    )  # float16[3]
    _pack_value(
        "h", "foc_offsets_voltage_undriven", scale=10000, is_array=True, array_len=3
    )  # float16[3]
    _pack_value("?", "foc_phase_filter_enable")  # bool (-> uint8_t)
    _pack_value("?", "foc_phase_filter_disable_fault")  # bool (-> uint8_t)
    _pack_value("f", "foc_phase_filter_max_erpm")  # float32
    _pack_value("B", "foc_mtpa_mode")  # uint8_t
    _pack_value("f", "foc_fw_current_max")  # float32
    _pack_value("h", "foc_fw_duty_start", scale=10000)  # float16
    _pack_value("h", "foc_fw_ramp_time", scale=1000)  # float16
    _pack_value("h", "foc_fw_q_current_factor", scale=10000)  # float16
    _pack_value("B", "foc_speed_soure")  # uint8_t (Typo in C?)
    _pack_value("?", "foc_short_ls_on_zero_duty")  # bool (-> uint8_t)

    if DEBUG_PACKER:
        print("\n--- [DEBUG] Packing Speed/Pos PID ---")
    _pack_value("B", "sp_pid_loop_rate")  # uint8_t
    _pack_value("f", "s_pid_kp")  # float32
    _pack_value("f", "s_pid_ki")  # float32
    _pack_value("f", "s_pid_kd")  # float32
    _pack_value("h", "s_pid_kd_filter", scale=10000)  # float16
    _pack_value("f", "s_pid_min_erpm")  # float32
    _pack_value("?", "s_pid_allow_braking")  # bool (-> uint8_t)
    _pack_value("f", "s_pid_ramp_erpms_s")  # float32
    _pack_value("B", "s_pid_speed_source")  # uint8_t
    _pack_value("f", "p_pid_kp")  # float32
    _pack_value("f", "p_pid_ki")  # float32
    _pack_value("f", "p_pid_kd")  # float32
    _pack_value("f", "p_pid_kd_proc")  # float32
    _pack_value("h", "p_pid_kd_filter", scale=10000)  # float16
    _pack_value("f", "p_pid_ang_div")  # float32
    _pack_value("h", "p_pid_gain_dec_angle", scale=10)  # float16
    _pack_value("f", "p_pid_offset")  # float32

    if DEBUG_PACKER:
        print("\n--- [DEBUG] Packing Current Controller ---")
    _pack_value("h", "cc_startup_boost_duty", scale=10000)  # float16
    _pack_value("f", "cc_min_current")  # float32
    _pack_value("f", "cc_gain")  # float32
    _pack_value("h", "cc_ramp_step_max", scale=10000)  # float16

    if DEBUG_PACKER:
        print("\n--- [DEBUG] Packing Misc Motor Settings ---")
    _pack_value("i", "m_fault_stop_time_ms")  # int32
    _pack_value("h", "m_duty_ramp_step", scale=10000)  # float16
    _pack_value("f", "m_current_backoff_gain")  # float32
    _pack_value("I", "m_encoder_counts")  # uint32
    _pack_value("h", "m_encoder_sin_amp", scale=1000)  # float16
    _pack_value("h", "m_encoder_cos_amp", scale=1000)  # float16
    _pack_value("h", "m_encoder_sin_offset", scale=1000)  # float16
    _pack_value("h", "m_encoder_cos_offset", scale=1000)  # float16
    _pack_value("h", "m_encoder_sincos_filter_constant", scale=1000)  # float16
    _pack_value("h", "m_encoder_sincos_phase_correction", scale=1000)  # float16
    _pack_value("B", "m_sensor_port_mode")  # uint8_t
    _pack_value("?", "m_invert_direction")  # bool (-> uint8_t)
    _pack_value("B", "m_drv8301_oc_mode")  # uint8_t
    _pack_value("B", "m_drv8301_oc_adj")  # uint8_t
    _pack_value("f", "m_bldc_f_sw_min")  # float32
    _pack_value("f", "m_bldc_f_sw_max")  # float32
    _pack_value("f", "m_dc_f_sw")  # float32
    _pack_value("f", "m_ntc_motor_beta")  # float32
    _pack_value("B", "m_out_aux_mode")  # uint8_t
    _pack_value("B", "m_motor_temp_sens_type")  # uint8_t
    _pack_value("f", "m_ptc_motor_coeff")  # float32
    _pack_value("h", "m_ntcx_ptcx_res", scale=0.1)  # float16 (!!! scale 0.1 !!!)
    _pack_value("h", "m_ntcx_ptcx_temp_base", scale=10)  # float16
    _pack_value("B", "m_hall_extra_samples")  # uint8_t
    _pack_value("B", "m_batt_filter_const")  # uint8_t

    if DEBUG_PACKER:
        print("\n--- [DEBUG] Packing SI Units ---")
    _pack_value("B", "si_motor_poles")  # uint8_t
    _pack_value("f", "si_gear_ratio")  # float32
    _pack_value("f", "si_wheel_diameter")  # float32
    _pack_value("B", "si_battery_type")  # uint8_t
    _pack_value("B", "si_battery_cells")  # uint8_t
    _pack_value("f", "si_battery_ah")  # float32
    _pack_value("f", "si_motor_nl_current")  # float32

    if DEBUG_PACKER:
        print("\n--- [DEBUG] Packing BMS Config ---")
    _pack_value("B", "bms.type")  # uint8_t
    _pack_value("B", "bms.limit_mode")  # uint8_t
    _pack_value("B", "bms.t_limit_start")  # uint8_t
    _pack_value("B", "bms.t_limit_end")  # uint8_t
    _pack_value("h", "bms.soc_limit_start", scale=1000)  # float16
    _pack_value("h", "bms.soc_limit_end", scale=1000)  # float16
    _pack_value("h", "bms.vmin_limit_start", scale=1000)  # float16
    _pack_value("h", "bms.vmin_limit_end", scale=1000)  # float16
    _pack_value("h", "bms.vmax_limit_start", scale=1000)  # float16
    _pack_value("h", "bms.vmax_limit_end", scale=1000)  # float16
    _pack_value("B", "bms.fwd_can_mode")  # uint8_t

    # --- 모든 필드 패킹 완료 ---
    final_size = len(packed_data)
    expected_approx_size = 477  # GET 응답 페이로드 크기 (Signature 제외) + Signature 크기(4) = 481? (정확히는 파서/C코드 비교필요)
    if DEBUG_PACKER:
        print("\n--- pack_mc_conf_serialized: Packing 완료 ---")
        print(f"최종 생성된 페이로드(Signature 포함) 크기: {final_size} 바이트")
        # 간단한 크기 검증 (GET 응답과 비슷해야 함)
        # if abs(final_size - expected_approx_size) > 10: # 예상 크기와 너무 다르면 경고
        #     print(f"!!![경고] 최종 페이로드 크기({final_size})가 예상 크기({expected_approx_size} 근처)와 많이 다릅니다! 모든 필드가 포함되었는지 확인하세요.")

    # 남아있는 키 확인 (디버깅)
    # remaining_keys = list(config_dict.keys())
    # if DEBUG_PACKER and remaining_keys:
    #     print(f"[DEBUG] 경고: 패킹 후 config_dict에 처리되지 않은 키가 남아있습니다: {remaining_keys}")

    return bytes(packed_data)  # 불변 바이트 객체 반환
