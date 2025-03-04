from etils import epath

ROOT_PATH = epath.Path(__file__).parents[2] / "robot_model" / "crab"
XML_PATH = ROOT_PATH / "crab_fewer_collisions.xml"


FEET_SITES = [
    "front_right",
    "front_left",
    "back_right",
    "back_left",
]


FEET_POS_SENSOR = [f"{site}_pos" for site in FEET_SITES]

ROOT_BODY = "base"
TORSO_BODY_ID = 1

UPVECTOR_SENSOR = "upvector"
GLOBAL_LINVEL_SENSOR = "global_linvel"
GLOBAL_ANGVEL_SENSOR = "global_angvel"
LOCAL_LINVEL_SENSOR = "local_linvel"
ACCELEROMETER_SENSOR = "accelerometer"
GYRO_SENSOR = "gyro"
ORIENTATION_SENSOR = "orientation"

kp = {"roll": 20, "pitch": 150, "wrist": 5}
INITIAL_JOINT_CONFIGURATION = [
    -1.434e-4,
    -2.827e-3,
    -5.650e-1,
    5.650e-1,
    5.650e-1,
    -5.650e-1,
    2.827e-3,
    5.574e-8,
    0.523,
    3.1517e-8,
    -1.57,
    1.555e-7,
    -4.075e-8,
    -5.106e-6,
    3.007e-6,
    -2.977e-3,
    -5.650e-1,
    5.650e-1,
    5.650e-1,
    -5.650e-1,
    -2.9767e-3,
    -2.347e-8,
    -0.523,
    4.1193e-8,
    -1.57,
    -6.215e-7,
    -3.4377e-7,
]
