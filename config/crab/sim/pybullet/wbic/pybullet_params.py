import numpy as np


class Config(object):
    CONTROLLER_DT = 0.001
    N_SUBSTEP = 1

    INITIAL_BASE_JOINT_POS = [0, 0, 0.35]
    INITIAL_BASE_JOINT_QUAT = [0, 0, 0, 1]
    INITIAL_CYLINDER_BASE_JOINT_POS = [-1.0, -1.0, 0.55]
    INITIAL_CYLINDER_BASE_JOINT_QUAT = [0.707, 0, 0, 0.707]

    PRINT_ROBOT_INFO = True

    MEASURE_COMPUTATION_TIME = False

    VIDEO_RECORD = False
    RECORD_FREQ = 50
    RENDER_WIDTH = 1920
    RENDER_HEIGHT = 1080

    ##TODO:
    USE_MESHCAT = False


class CrabLinkIdx(object):
    base = -1
    # 776
    FR_hip = 0
    # second hip joint here
    FR_upperthigh = 2
    FR_upperthigh_proximal = 3
    FR_upperthigh_distal = 4
    # FR_upperthigh_end = 5
    # 665
    FR_lowerthigh = 6
    FR_lowerthigh_proximal = 7
    FR_lowerthigh_distal = 8
    # FR_lowerthigh_end = 9
    # 5555
    FR_uppercalf = 10
    FR_uppercalf_proximal = 11
    FR_uppercalf_distal = 12
    FR_lowercalf = 13
    FR_wrist = 14
    FR_foot = 15

    # FL_ version (incremented by 16)
    FL_hip = 16
    FL_upperthigh = 18
    FL_upperthigh_proximal = 19
    FL_upperthigh_distal = 20
    # FL_upperthigh_end = 21
    # 665
    FL_lowerthigh = 22
    FL_lowerthigh_proximal = 23
    FL_lowerthigh_distal = 24
    # FL_lowerthigh_end = 25
    # 5555
    FL_uppercalf = 26
    FL_uppercalf_proximal = 27
    FL_uppercalf_distal = 28
    FL_lowercalf = 29
    FL_wrist = 30
    FL_foot = 31

    # RR_ version (incremented by another 16)
    RR_hip = 32
    RR_upperthigh = 34
    RR_upperthigh_proximal = 35
    RR_upperthigh_distal = 36
    # RR_upperthigh_end = 37
    # 665
    RR_lowerthigh = 38
    RR_lowerthigh_proximal = 39
    RR_lowerthigh_distal = 40
    # RR_lowerthigh_end = 41
    # 5555
    RR_uppercalf = 42
    RR_uppercalf_proximal = 43
    RR_uppercalf_distal = 44
    RR_lowercalf = 45
    RR_wrist = 46
    RR_foot = 47

    # RL_ version (incremented by another 16)
    RL_hip = 48
    RL_upperthigh = 50
    RL_upperthigh_proximal = 51
    RL_upperthigh_distal = 52
    # RL_upperthigh_end = 53
    # 665
    RL_lowerthigh = 54
    RL_lowerthigh_proximal = 55
    RL_lowerthigh_distal = 56
    # RL_lowerthigh_end = 57
    # 5555
    RL_uppercalf = 58
    RL_uppercalf_proximal = 59
    RL_uppercalf_distal = 60
    RL_lowercalf = 61
    RL_wrist = 62
    RL_foot = 63

    imu = 64


class CrabJointIdx(object):
    FR_hip_pitch = 0
    FR_hip_yaw = 1
    # fixed = 2
    FR_upperlowerthigh_roll = 3
    FR_upperlowerthigh_pitch = 4
    # fixed = 5, 6
    FR_thighcalf_roll = 7
    FR_thighcalf_pitch = 8
    # fixed = 9, 10
    FR_upperlowercalf_roll = 11
    FR_upperlowercalf_pitch = 12
    # fixed = 13
    FR_wrist_joint = 14
    # fixed = 15

    FL_hip_pitch = 16
    FL_hip_yaw = 17
    # fixed = 18
    FL_upperlowerthigh_roll = 19
    FL_upperlowerthigh_pitch = 20
    # fixed = 21, 22
    FL_thighcalf_roll = 23
    FL_thighcalf_pitch = 24
    # fixed = 25, 26
    FL_upperlowercalf_roll = 27
    FL_upperlowercalf_pitch = 28
    # fixed = 29
    FL_wrist_joint = 30
    # fixed = 31

    RR_hip_pitch = 32
    RR_hip_yaw = 33
    # fixed = 34
    RR_upperlowerthigh_roll = 35
    RR_upperlowerthigh_pitch = 36
    # fixed = 37, 38
    RR_thighcalf_roll = 39
    RR_thighcalf_pitch = 40
    # fixed = 41, 42
    RR_upperlowercalf_roll = 43
    RR_upperlowercalf_pitch = 44
    # fixed = 45
    RR_wrist_joint = 46
    # fixed = 47

    RL_hip_pitch = 48
    RL_hip_yaw = 49
    # fixed = 50
    RL_upperlowerthigh_roll = 51
    RL_upperlowerthigh_pitch = 52
    # fixed = 53, 54
    RL_thighcalf_roll = 55
    RL_thighcalf_pitch = 56
    # fixed = 57, 58
    RL_upperlowercalf_roll = 59
    RL_upperlowercalf_pitch = 60
    # fixed = 61
    RL_wrist_joint = 62
    # fixed = 63


class JointGains(object):
    kp = 5.0 * np.ones(63)
    kd = 0.01 * np.ones(63)
