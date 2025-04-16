# --- vesc_protocol_utils.py ---

import struct
import traceback

# !!!!! 경로 확인 필요: frame 함수 위치 !!!!!
try:
    from pyvesc.protocol.packet.codec import frame
except ImportError:
    # 대체 경로 시도
    try:
        from pyvesc.protocol.packet.codec import frame
    except ImportError as e:
        print(
            f"오류(vesc_protocol_utils): 'frame' 함수 import 실패. pyvesc 라이브러리 구조 확인 필요. ({e})"
        )
        raise

# 필요한 메시지 클래스 및 페이로드 생성 함수 import
# !!!!! 아래 경로는 실제 프로젝트 구조에 맞게 조정 필요 !!!!!
try:
    from pyvesc.VESC.messages.setters import SetMcConf, DetectApplyAllFOC

    # pack_mc_conf_serialized 함수가 있는 위치에서 import
    from pyvesc.VESC.messages.parser import pack_mc_conf_serialized
except ImportError as e:
    print(
        f"오류(vesc_protocol_utils): 필요한 클래스/함수 import 실패. 경로 확인 필요. ({e})"
    )
    raise


def encode_set_mcconf(message: SetMcConf):
    """
    SetMcConf 메시지(ID 13)를 VESC 통신 패킷으로 인코딩합니다.
    pack_mc_conf_serialized 함수와 frame 함수를 사용합니다.
    """
    if not isinstance(message, SetMcConf):
        raise TypeError("...")
    if not hasattr(message, "mc_configuration"):
        raise AttributeError("...")

    try:
        # print("  (Util) Packer 함수(pack_mc_conf_serialized) 호출...") # 로그 줄임
        data_payload = pack_mc_conf_serialized(message.mc_configuration)
        if not data_payload:
            raise ValueError("Packer 빈 데이터 반환")

        command_id_byte = bytes([message.id])  # ID: 13
        full_payload = command_id_byte + data_payload

        # print(f"  (Util) pyvesc.protocol.packet.codec.frame 함수 호출...") # 로그 줄임
        final_packet = frame(full_payload)
        return final_packet
    except KeyError as e:
        print(f"!!! (Util) 패킹 오류 (KeyError): '{e}'.")
        raise
    except Exception as e:
        print(f"!!! (Util) SetMcConf 인코딩 오류: {e}")
        traceback.print_exc()
        raise


def encode_detect_apply_all_foc(message: DetectApplyAllFOC):
    """
    DetectApplyAllFOC 메시지(ID 58)를 VESC 통신 패킷으로 수동 인코딩합니다.
    """
    if not isinstance(message, DetectApplyAllFOC):
        raise TypeError("...")
    if not hasattr(message, "fields") or len(message.fields) != 6:
        raise ValueError("...")

    payload_fmt = ">"
    values_to_pack = []
    try:
        # print("  (Util) DetectApplyAllFOC 수동 패킹 시작...") # 로그 줄임
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

        data_payload = struct.pack(payload_fmt, *values_to_pack)

        command_id_byte = bytes([message.id])  # 예: ID 58
        full_payload = command_id_byte + data_payload

        # print(f"  (Util) pyvesc.protocol.packet.codec.frame 함수 호출...") # 로그 줄임
        final_packet = frame(full_payload)
        return final_packet

    except AttributeError as e:
        raise AttributeError(f"메시지 필드 '{e}' 없음.")
    except (struct.error, TypeError, ValueError) as e:
        print(f"!!! (Util) 패킹 오류: {e}")
        raise
    except Exception as e:
        print(f"!!! (Util) DetectApplyAllFOC 인코딩 오류: {e}")
        traceback.print_exc()
        raise


# --- 다른 필요한 인코딩 헬퍼 함수들 추가 가능 ---
