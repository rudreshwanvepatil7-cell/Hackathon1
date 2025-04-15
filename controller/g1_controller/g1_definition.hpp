#pragma once

namespace g1_link {
constexpr int pelvis = 2;
constexpr int left_hip_pitch_link = 4;
constexpr int left_hip_roll_link = 6;
constexpr int left_hip_yaw_link = 8;
constexpr int left_knee_link = 10;
constexpr int left_ankle_pitch_link = 12;
constexpr int left_ankle_roll_link = 14;
constexpr int l_foot_contact = 16;
constexpr int pelvis_contour_link = 18;
constexpr int right_hip_pitch_link = 20;
constexpr int right_hip_roll_link = 22;
constexpr int right_hip_yaw_link = 24;
constexpr int right_knee_link = 26;
constexpr int right_ankle_pitch_link = 28;
constexpr int right_ankle_roll_link = 30;
constexpr int r_foot_contact = 32;
constexpr int torso_link = 34;
constexpr int head_link = 36;
constexpr int imu_link = 38;
constexpr int left_shoulder_pitch_link = 40;
constexpr int left_shoulder_roll_link = 42;
constexpr int left_shoulder_yaw_link = 44;
constexpr int left_elbow_pitch_link = 46;
constexpr int left_elbow_roll_link = 48;
constexpr int left_palm_link = 50;
constexpr int left_five_link = 52;
constexpr int left_six_link = 54;
constexpr int left_three_link = 56;
constexpr int left_four_link = 58;
constexpr int left_zero_link = 60;
constexpr int left_one_link = 62;
constexpr int left_two_link = 64;
constexpr int logo_link = 66;
constexpr int right_shoulder_pitch_link = 68;
constexpr int right_shoulder_roll_link = 70;
constexpr int right_shoulder_yaw_link = 72;
constexpr int right_elbow_pitch_link = 74;
constexpr int right_elbow_roll_link = 76;
constexpr int right_palm_link = 78;
constexpr int right_five_link = 80;
constexpr int right_six_link = 82;
constexpr int right_three_link = 84;
constexpr int right_four_link = 86;
constexpr int right_zero_link = 88;
constexpr int right_one_link = 90;
constexpr int right_two_link = 92;
constexpr int torso_com_link = 94;
} // namespace g1_link

namespace g1_joint {
constexpr int left_hip_pitch_joint = 0;
constexpr int left_hip_roll_joint = 1;
constexpr int left_hip_yaw_joint = 2;
constexpr int left_knee_joint = 3;
constexpr int left_ankle_pitch_joint = 4;
constexpr int left_ankle_roll_joint = 5;
constexpr int right_hip_pitch_joint = 6;
constexpr int right_hip_roll_joint = 7;
constexpr int right_hip_yaw_joint = 8;
constexpr int right_knee_joint = 9;
constexpr int right_ankle_pitch_joint = 10;
constexpr int right_ankle_roll_joint = 11;
constexpr int torso_joint = 12;
constexpr int left_shoulder_pitch_joint = 13;
constexpr int left_shoulder_roll_joint = 14;
constexpr int left_shoulder_yaw_joint = 15;
constexpr int left_elbow_pitch_joint = 16;
constexpr int left_elbow_roll_joint = 17;
constexpr int left_five_joint = 18;
constexpr int left_six_joint = 19;
constexpr int left_three_joint = 20;
constexpr int left_four_joint = 21;
constexpr int left_zero_joint = 22;
constexpr int left_one_joint = 23;
constexpr int left_two_joint = 24;
constexpr int right_shoulder_pitch_joint = 25;
constexpr int right_shoulder_roll_joint = 26;
constexpr int right_shoulder_yaw_joint = 27;
constexpr int right_elbow_pitch_joint = 28;
constexpr int right_elbow_roll_joint = 29;
constexpr int right_five_joint = 30;
constexpr int right_six_joint = 31;
constexpr int right_three_joint = 32;
constexpr int right_four_joint = 33;
constexpr int right_zero_joint = 34;
constexpr int right_one_joint = 35;
constexpr int right_two_joint = 36;
} // namespace g1_joint

namespace g1 {
constexpr int n_qdot = 43;
constexpr int n_adof = 37;
} // namespace g1
