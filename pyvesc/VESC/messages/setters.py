from pyvesc.protocol.base import VESCMessage
from pyvesc.protocol.interface import encode
from pyvesc.VESC.messages import VedderCmd


class SetDutyCycle(metaclass=VESCMessage):
    """Set the duty cycle.

    :ivar duty_cycle: Value of duty cycle to be set (range [-1e5, 1e5]).
    """

    id = VedderCmd.COMM_SET_DUTY
    fields = [("duty_cycle", "i", 100000)]


class SetRPM(metaclass=VESCMessage):
    """Set the RPM.

    :ivar rpm: Value to set the RPM to.
    """

    id = VedderCmd.COMM_SET_RPM
    fields = [("rpm", "i")]


class SetCurrent(metaclass=VESCMessage):
    """Set the current (in milliamps) to the motor.

    :ivar current: Value to set the current to (in milliamps).
    """

    id = VedderCmd.COMM_SET_CURRENT
    fields = [("current", "i", 1000)]


class SetCurrentBrake(metaclass=VESCMessage):
    """Set the current brake (in milliamps).

    :ivar current_brake: Value to set the current brake to (in milliamps).
    """

    id = VedderCmd.COMM_SET_CURRENT_BRAKE
    fields = [("current_brake", "i", 1000)]


class SetPosition(metaclass=VESCMessage):
    """Set the rotor angle based off of an encoder or sensor

    :ivar pos: Value to set the current position or angle to.
    """

    id = VedderCmd.COMM_SET_POS
    fields = [("pos", "i", 1000000)]


class SetRotorPositionMode(metaclass=VESCMessage):
    """Sets the rotor position feedback mode.

    It is reccomended to use the defined modes as below:
        * DISP_POS_OFF
        * DISP_POS_MODE_ENCODER
        * DISP_POS_MODE_PID_POS
        * DISP_POS_MODE_PID_POS_ERROR

    :ivar pos_mode: Value of the mode
    """

    DISP_POS_OFF = 0
    DISP_POS_MODE_ENCODER = 3
    DISP_POS_MODE_PID_POS = 4
    DISP_POS_MODE_PID_POS_ERROR = 5

    id = VedderCmd.COMM_SET_DETECT
    fields = [("pos_mode", "b")]


class SetServoPosition(metaclass=VESCMessage):
    """Sets the position of s servo connected to the VESC.

    :ivar servo_pos: Value of position (range [0, 1])
    """

    id = VedderCmd.COMM_SET_SERVO_POS
    fields = [("servo_pos", "h", 1000)]


class Alive(metaclass=VESCMessage):
    """Heartbeat signal to keep VESC alive"""

    id = VedderCmd.COMM_ALIVE
    fields = []


class SetMcConf(metaclass=VESCMessage):
    id = VedderCmd.COMM_SET_MCCONF  # 1단계에서 추가한 ID
    fields = []


class DetectMotorRL(metaclass=VESCMessage):
    """모터 저항(R)과 인덕턴스(L) 측정을 시작합니다. (입력 페이로드 없음)"""

    id = VedderCmd.COMM_DETECT_MOTOR_R_L
    fields = []  # 입력 페이로드 없음


class DetectMotorFluxLinkage(metaclass=VESCMessage):
    """
    모터 역기전력 상수(Flux Linkage) 측정을 시작합니다.
    !!! 이 명령은 모터를 회전시킵니다 !!!
    """

    id = VedderCmd.COMM_DETECT_MOTOR_FLUX_LINKAGE
    # 입력: current(A*1e3), min_rpm(RPM*1e3), duty(Ratio*1e3), resistance(Ohm*1e6) -> int32 가정
    fields = [
        ("current", "i", 1000),
        ("min_rpm", "i", 1000),
        ("duty", "i", 1000),
        ("resistance", "i", 1000000),
    ]


class DetectMotorParam(metaclass=VESCMessage):
    """범용 모터 파라미터 감지를 시작합니다."""

    id = VedderCmd.COMM_DETECT_MOTOR_PARAM
    # 입력: current(A*1e3), min_rpm(RPM*1e3), low_duty(Ratio*1e3) -> int32 가정
    fields = [("current", "i", 1000), ("min_rpm", "i", 1000), ("low_duty", "i", 1000)]


class DetectMotorFluxLinkageOpenLoop(metaclass=VESCMessage):
    """오픈루프 방식으로 Flux Linkage 측정을 시작합니다."""

    id = VedderCmd.COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP  # ID 57 사용
    # 입력 파라미터: current, duty, erpm_per_sec, res, ind
    # VESC C 코드의 buffer_get_float32 스케일링 확인 필요 (int32, 스케일 가정)
    fields = [
        ("current", "i", 1000),  # 스케일 1e3
        ("duty", "i", 1000),  # 스케일 1e3
        ("erpm_per_sec", "i", 1000),  # 스케일 1e3
        ("resistance", "i", 1000000),  # 스케일 1e6 (uOhm)
        ("inductance", "i", 100000000),  # !!!!! 스케일 1e8 !!!!!
    ]


class DetectApplyAllFOC(metaclass=VESCMessage):
    """FOC 센서리스 모드에서 모든 감지 파라미터를 적용합니다."""

    id = VedderCmd.COMM_DETECT_APPLY_ALL_FOC  # 예시 ID. 실제 값 확인!

    # 페이로드 필드 정의 (C 코드 기반)
    # buffer_get_float32(data, scale, &ind) -> Python에서는 float 값을 scale 곱해서 int32('i')로 보냄
    fields = [
        # !!!!! 수정: bool 타입 필드에도 스케일 placeholder 추가 (None 또는 0) !!!!!
        ("detect_can", "?", None),
        ("max_power_loss", "i", 1000),
        ("min_current_in", "i", 1000),
        ("max_current_in", "i", 1000),
        ("openloop_rpm", "i", 1000),
        ("sl_erpm", "i", 1000),
    ]


class SetAppConf(metaclass=VESCMessage):
    """어플리케이션 설정(APPCONF)을 VESC에 씁니다."""

    id = VedderCmd.COMM_SET_APPCONF
    fields = []
