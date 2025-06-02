import struct
from enum import Enum, auto


class Keys(Enum):
    MESSAGE_SIZE = auto()
    TIME = auto()
    Q_TARGET = auto()
    QD_TARGET = auto()
    QDD_TARGET = auto()
    I_TARGET = auto()
    M_TARGET = auto()
    Q_ACTUAL = auto()
    QD_ACTUAL = auto()
    I_ACTUAL = auto()
    I_CONTROL = auto()
    TOOL_VECTOR_ACTUAL = auto()
    TCP_SPEED_ACTUAL = auto()
    TCP_FORCE = auto()
    TOOL_VECTOR_TARGET = auto()
    TCP_SPEED_TARGET = auto()
    DIGITAL_INPUT_BITS = auto()
    MOTOR_TEMPERATURES = auto()
    CONTROLLER_TIMER = auto()
    TEST_VALUE = auto()
    ROBOT_MODE = auto()
    JOINT_MODES = auto()
    SAFETY_MODE = auto()
    TOOL_ACCELEROMETER_VALUES = auto()
    SPEED_SCALING = auto()
    LINEAR_MOMENTUM_NORM = auto()
    V_MAIN = auto()
    V_ROBOT = auto()
    I_ROBOT = auto()
    V_ACTUAL = auto()
    DIGITAL_OUTPUTS = auto()
    PROGRAM_STATE = auto()
    ELBOW_POSITION = auto()
    ELBOW_VELOCITY = auto()
    SAFETY_STATUS = auto()


pos_dict = {
    # (data type, size in bytes, position of start byte)
    Keys.MESSAGE_SIZE:          ('>i', 4, 0),
    Keys.TIME:                  ('!d', 8, 4),
    Keys.Q_TARGET:              ('!dddddd', 48, 12),    # base, shoulder, elbow, wrist1, wrist2, wrist3 [rad]
    Keys.QD_TARGET:             ('!dddddd', 48, 60),
    Keys.QDD_TARGET:            ('!dddddd', 48, 108),
    Keys.I_TARGET:              ('!dddddd', 48, 156),
    Keys.M_TARGET:              ('!dddddd', 48, 204),
    Keys.Q_ACTUAL:              ('!dddddd', 48, 252),
    Keys.QD_ACTUAL:             ('!dddddd', 48, 300),
    Keys.I_ACTUAL:              ('!dddddd', 48, 348),
    Keys.I_CONTROL:             ('!dddddd', 48, 396),
    Keys.TOOL_VECTOR_ACTUAL:    ('!dddddd', 48, 444),   # x,y,z [m] and rx, ry, rz [rad]
    Keys.TCP_SPEED_ACTUAL:      ('!dddddd', 48, 492),   # speed in x,y,z [m/s] and rotation speed [rad/s] in rx,ry,rz direction
    Keys.TCP_FORCE:             ('!dddddd', 48, 540),
    Keys.TOOL_VECTOR_TARGET:    ('!dddddd', 48, 588),
    Keys.TCP_SPEED_TARGET:      ('!dddddd', 48, 636),
    Keys.DIGITAL_INPUT_BITS:    ('!d', 8, 684),
    Keys.MOTOR_TEMPERATURES:    ('!dddddd', 48, 692),
    Keys.CONTROLLER_TIMER:      ('!d', 8, 740),
    Keys.TEST_VALUE:            ('!d', 8, 748),
    Keys.ROBOT_MODE:            ('!d', 8, 756),
    Keys.JOINT_MODES:           ('!dddddd', 48, 764),
    Keys.SAFETY_MODE:           ('!d', 8, 812),
    Keys.TOOL_ACCELEROMETER_VALUES: ('!ddd', 24, 868),
    Keys.SPEED_SCALING:         ('!d', 8, 940),
    Keys.LINEAR_MOMENTUM_NORM:  ('!d', 8, 948),
    Keys.V_MAIN:                ('!d', 8, 972),
    Keys.V_ROBOT:               ('!d', 8, 980),
    Keys.I_ROBOT:               ('!d', 8, 988),
    Keys.V_ACTUAL:              ('!dddddd', 48, 996),
    Keys.DIGITAL_OUTPUTS:       ('!d', 8, 1044),
    Keys.PROGRAM_STATE:         ('!d', 8, 1052),
    Keys.ELBOW_POSITION:        ('!ddd', 24, 1060),
    Keys.ELBOW_VELOCITY:        ('!ddd', 24, 1084),
    Keys.SAFETY_STATUS:         ('!d', 8, 1108),
}


def unpack(r: bytes, name: Keys):
    if name in pos_dict.keys():
        return struct.unpack(pos_dict[name][0], r[pos_dict[name][2]:pos_dict[name][2]+pos_dict[name][1]])
    else:
        raise KeyError(f'{name} is not in keys of position dict for data pack mapping of UR Robot')


def unpack_batch(r: bytes, names: list[Keys]):
    return [unpack(r, name) for name in names]


def unpack_all(r: bytes):
    return {key.name: unpack(r, key) for key in Keys}
