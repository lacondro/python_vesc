# 예: pyvesc/VESC/messages/vesc_parser.py
import struct
import pprint


def parse_mc_conf_serialized(payload_buffer):
    """
    VESC 6.05 펌웨어의 confgenerator_serialize_mcconf 함수 로직에 맞춰 파싱합니다.
    (이전 코드에서 완성된 parse_mc_conf_serialized 함수 내용 전체를 여기에 붙여넣습니다)
    """
    parsed_config = {}
    offset = 0
    buffer_len = len(payload_buffer)
    # print(f"mc_configuration (Serialized) 파싱 시작...") # 로그 레벨 조절 가능

    # 내부 헬퍼 함수 정의
    def unpack_and_advance(fmt, field_name, scale=None):
        # ... (이전 코드와 동일한 unpack_and_advance 함수 내용) ...
        nonlocal offset
        try:
            full_fmt = ">" + fmt
            size = struct.calcsize(full_fmt)
            if offset + size > buffer_len:
                if field_name != "crc":
                    # print(f"\n오류: 필드 '{field_name}' 파싱 중 버퍼 길이 초과...") # 로그 레벨 조절
                    raise IndexError(
                        f"Buffer length exceeded parsing field '{field_name}'"
                    )
                else:
                    return None  # CRC 길이 부족
            value = struct.unpack_from(full_fmt, payload_buffer, offset)[0]
            if scale is not None and scale != 0:
                value = float(value) / scale
            parsed_config[field_name] = value
            offset += size
            return value
        except (struct.error, IndexError) as e:
            # print(f"\n오류: 필드 '{field_name}' 파싱 중 오류 발생...") # 로그 레벨 조절
            raise
        except Exception as e:
            # print(f"\n예상치 못한 오류: 필드 '{field_name}' 파싱 중: {e}") # 로그 레벨 조절
            raise

    try:
        # --- confgenerator_serialize_mcconf 순서대로 파싱 ---
        # 1. Signature
        unpack_and_advance("I", "MCCONF_SIGNATURE")
        # 2. Enum/bool
        unpack_and_advance("B", "pwm_mode")
        unpack_and_advance("B", "comm_mode")
        # ... (이전 코드에서 완성된 모든 필드 파싱 로직 붙여넣기) ...
        # Limits ... FOC ... PID ... Misc ... Setup ... BMS ...

        # --- BMS 끝, CRC 파싱 시도 ---
        # print(f"\nBMS 필드까지 파싱 완료. 현재 오프셋: {offset}") # 로그 레벨 조절
        remaining_bytes = buffer_len - offset
        # print(f"남은 바이트 수: {remaining_bytes}") # 로그 레벨 조절

        if remaining_bytes >= 2:
            crc_offset = buffer_len - 2
            try:
                crc_fmt = ">H"
                crc_value = struct.unpack_from(crc_fmt, payload_buffer, crc_offset)[0]
                parsed_config["crc"] = crc_value
                offset = buffer_len
                # print(f"마지막 2바이트를 CRC로 파싱 성공...") # 로그 레벨 조절
            except (struct.error, IndexError) as e:
                print(
                    f"\n오류: 마지막 2바이트 CRC 파싱 실패. {e}"
                )  # 이 오류는 유지하는 것이 좋음
        # ... (CRC 관련 경고 메시지 유지) ...

        # print(f"\n파싱 완료! 최종 오프셋: {offset} / 페이로드 길이: {buffer_len}") # 로그 레벨 조절
        if offset != buffer_len:
            print(
                "경고: 파싱 완료 후 오프셋과 페이로드 길이가 일치하지 않습니다!"
            )  # 경고 유지

        return parsed_config

    except (struct.error, IndexError) as e:
        print(f"\n파싱 중단됨: {e}")  # 오류 유지
        return parsed_config  # 부분 결과 반환
    except Exception as e:
        print(f"\n예상치 못한 파싱 오류 발생: {e}")  # 오류 유지
        return parsed_config
