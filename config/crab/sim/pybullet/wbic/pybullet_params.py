import numpy as np


class Config(object):
    CONTROLLER_DT = 0.001
    N_SUBSTEP = 1

    INITIAL_BASE_JOINT_POS = [0, 0, 0.35]
    INITIAL_BASE_JOINT_QUAT = [0, 0, 0, 1]
    INITIAL_CYLINDER_BASE_JOINT_POS = [-1., -1., 0.55]
    INITIAL_CYLINDER_BASE_JOINT_QUAT = [0.707, 0, 0, 0.707]

    PRINT_ROBOT_INFO = True

    MEASURE_COMPUTATION_TIME = False

    VIDEO_RECORD = False
    RECORD_FREQ = 50
    RENDER_WIDTH = 1920
    RENDER_HEIGHT = 1080

    ##TODO:
    USE_MESHCAT = False


class crab_link_idx(object):
    base_link = 0
    top_lidar_link = 1
    bottom_lidar_link = 2
    back__sensor_link = 3
    left__sensor_link = 4
    right__sensor_link = 5
    front_right__sensor_link = 6
    front_left__sensor_link = 7
    front_right__base_link = 8
    front_right__cluster_1_base_link = 9
    front_right__cluster_1_proximal_link = 10
    front_right__cluster_1_distal_link = 11
    front_right__cluster_1_end_link = 12
    front_right__cluster_2_base_link = 13
    front_right__cluster_2_proximal_link = 14
    front_right__cluster_2_distal_link = 15
    front_right__cluster_2_end_link = 16
    front_right__cluster_3_base_link = 17
    front_right__cluster_3_proximal_link = 18
    front_right__cluster_3_distal_link = 19
    front_right__cluster_3_mid_link = 20
    front_right__cluster_3_wrist_link = 21
    front_right__cluster_3_end_link = 22
    front_right__foot_link = 23
    front_left__base_link = 24
    front_left__cluster_1_base_link = 25
    front_left__cluster_1_proximal_link = 26
    front_left__cluster_1_distal_link = 27
    front_left__cluster_1_end_link = 28
    front_left__cluster_2_base_link = 29
    front_left__cluster_2_proximal_link = 30
    front_left__cluster_2_distal_link = 31
    front_left__cluster_2_end_link = 32
    front_left__cluster_3_base_link = 33
    front_left__cluster_3_proximal_link = 34
    front_left__cluster_3_distal_link = 35
    front_left__cluster_3_mid_link = 36
    front_left__cluster_3_wrist_link = 37
    front_left__cluster_3_end_link = 38
    front_left__foot_link = 39
    back_right__base_link = 40
    back_right__cluster_1_base_link = 41
    back_right__cluster_1_proximal_link = 42
    back_right__cluster_1_distal_link = 43
    back_right__cluster_1_end_link = 44
    back_right__cluster_2_base_link = 45
    back_right__cluster_2_proximal_link = 46
    back_right__cluster_2_distal_link = 47
    back_right__cluster_2_end_link = 48
    back_right__cluster_3_base_link = 49
    back_right__cluster_3_proximal_link = 50
    back_right__cluster_3_distal_link = 51
    back_right__cluster_3_mid_link = 52
    back_right__cluster_3_wrist_link = 53
    back_right__cluster_3_end_link = 54
    back_right__foot_link = 55
    back_left__base_link = 56
    back_left__cluster_1_base_link = 57
    back_left__cluster_1_proximal_link = 58
    back_left__cluster_1_distal_link = 59
    back_left__cluster_1_end_link = 60
    back_left__cluster_2_base_link = 61
    back_left__cluster_2_proximal_link = 62
    back_left__cluster_2_distal_link = 63
    back_left__cluster_2_end_link = 64
    back_left__cluster_3_base_link = 65
    back_left__cluster_3_proximal_link = 66
    back_left__cluster_3_distal_link = 67
    back_left__cluster_3_mid_link = 68
    back_left__cluster_3_wrist_link = 69
    back_left__cluster_3_end_link = 70
    back_left__foot_link = 71


class crab_joint_idx(object):
    FL_hip_joint = 2
    FL_thigh_joint = 3
    FL_calf_joint = 4
    FR_hip_joint = 8
    FR_thigh_joint = 9
    FR_calf_joint = 10
    RL_hip_joint = 14
    RL_thigh_joint = 15
    RL_calf_joint = 16
    RR_hip_joint = 20
    RR_thigh_joint = 21
    RR_calf_joint = 22


class joint_gains(object):
    # kp = 5. * np.ones(27)
    # kd = 0. * np.ones(27)
    kp = np.array([
        10, 10, 10, 10, 10, 10, 10, 5, 5, 5, 5, 5, 5, 5, 10, 10, 10, 10, 10,
        10, 10, 5, 5, 5, 5, 5, 5
    ])
    kd = np.array([
        0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.001, 0.001, 0.001, 0.001,
        0.001, 0.001, 0.01, 0.01, 0.001, 0.01, 0.01, 0.01, 0.01, 0.01, 0.001,
        0.001, 0.001, 0.001, 0.001, 0.001
    ])
