// # ---------------------------------- 
// # PYBULLET 
// # ---------------------------------- 

namespace crab_link {
constexpr int base_link = 4;                                // in pybullet 
constexpr int back__sensor_link = 6;                        // in pybullet 
constexpr int back_left__base_link = 8;                     // in pybullet 
constexpr int back_left__cluster_1_base_link = 10;          // in pybullet 
constexpr int back_left__cluster_1_proximal_link = 12;      // in pybullet 
constexpr int back_left__cluster_1_distal_link = 14;        // in pybullet 
constexpr int back_left__cluster_1_end_link = 16;           // in pybullet 
constexpr int back_left__cluster_2_base_link = 18;          // in pybullet 
constexpr int back_left__cluster_2_proximal_link = 20;      // in pybullet 
constexpr int back_left__cluster_2_distal_link = 22;        // in pybullet 
constexpr int back_left__cluster_2_end_link = 24;           // in pybullet 
constexpr int back_left__cluster_3_base_link = 26;          // in pybullet 
constexpr int back_left__cluster_3_proximal_link = 28;      // in pybullet 
constexpr int back_left__cluster_3_distal_link = 30;        // in pybullet 
constexpr int back_left__cluster_3_mid_link = 32;           // in pybullet   
constexpr int back_left__cluster_3_wrist_link = 34;         // in pybullet 
constexpr int back_left__cluster_3_end_link = 36;           // in pybullet 
constexpr int back_left__foot_link = 38;                    // in pybullet 
constexpr int back_right__base_link = 40;                   // in pybullet 
constexpr int back_right__cluster_1_base_link = 42;         // in pybullet 
constexpr int back_right__cluster_1_proximal_link = 44;     // in pybullet 
constexpr int back_right__cluster_1_distal_link = 46;       // in pybullet 
constexpr int back_right__cluster_1_end_link = 48;          // in pybullet 
constexpr int back_right__cluster_2_base_link = 50;         // in pybullet 
constexpr int back_right__cluster_2_proximal_link = 52;     // in pybullet 
constexpr int back_right__cluster_2_distal_link = 54;       // in pybullet 
constexpr int back_right__cluster_2_end_link = 56;          // in pybullet 
constexpr int back_right__cluster_3_base_link = 58;         // in pybullet 
constexpr int back_right__cluster_3_proximal_link = 60;     // in pybullet 
constexpr int back_right__cluster_3_distal_link = 62;       // in pybullet 
constexpr int back_right__cluster_3_mid_link = 64;          // in pybullet 
constexpr int back_right__cluster_3_wrist_link = 66;        // in pybullet 
constexpr int back_right__cluster_3_end_link = 68;          // in pybullet 
constexpr int back_right__foot_link = 70;                   // in pybullet 
constexpr int front_left__base_link = 72;                   // in pybullet
constexpr int front_left__cluster_1_base_link = 74;         // in pybullet 
constexpr int front_left__cluster_1_proximal_link = 76;     // in pybullet 
constexpr int front_left__cluster_1_distal_link = 78;       // in pybullet 
constexpr int front_left__cluster_1_end_link = 80;          // in pybullet 
constexpr int front_left__cluster_2_base_link = 82;         // in pybullet 
constexpr int front_left__cluster_2_proximal_link = 84;     // in pybullet 
constexpr int front_left__cluster_2_distal_link = 86;       // in pybullet 
constexpr int front_left__cluster_2_end_link = 88;          // in pybullet 
constexpr int front_left__cluster_3_base_link = 90;         // in pybullet 
constexpr int front_left__cluster_3_proximal_link = 92;     // in pybullet 
constexpr int front_left__cluster_3_distal_link = 94;       // in pybullet 
constexpr int front_left__cluster_3_mid_link = 96;          // in pybullet 
constexpr int front_left__cluster_3_wrist_link = 98;        // in pybullet 
constexpr int front_left__cluster_3_end_link = 100;         // in pybullet 
constexpr int front_left__foot_link = 102;                  // in pybullet 
constexpr int front_left__sensor_link = 104;                // in pybullet 
constexpr int front_right__base_link = 106;                 // in pybullet 
constexpr int front_right__cluster_1_base_link = 108;       // in pybullet 
constexpr int front_right__cluster_1_proximal_link = 110;   // in pybullet 
constexpr int front_right__cluster_1_distal_link = 112;     // in pybullet 
constexpr int front_right__cluster_1_end_link = 114;        // in pybullet 
constexpr int front_right__cluster_2_base_link = 116;       // in pybullet 
constexpr int front_right__cluster_2_proximal_link = 118;   // in pybullet 
constexpr int front_right__cluster_2_distal_link = 120;     // in pybullet 
constexpr int front_right__cluster_2_end_link = 122;        // in pybullet 
constexpr int front_right__cluster_3_base_link = 124;       // in pybullet 
constexpr int front_right__cluster_3_proximal_link = 126;   // in pybullet 
constexpr int front_right__cluster_3_distal_link = 128;     // in pybullet 
constexpr int front_right__cluster_3_mid_link = 130;        // in pybullet 
constexpr int front_right__cluster_3_wrist_link = 132;      // in pybullet 
constexpr int front_right__cluster_3_end_link = 134;        // in pybullet 
constexpr int front_right__foot_link = 136;                 // in pybullet    
constexpr int front_right__sensor_link = 138;               // in pybullet 
constexpr int left__sensor_link = 140;                      // in pybullet 
constexpr int right__sensor_link = 142;                     // in pybullet 
constexpr int bottom_lidar_link = 144;                      // in pybullet 
constexpr int top_lidar_link = 146;                         // in pybullet 
} // namespace crab_link 

namespace crab_joint {
constexpr int back_left__cluster_1_roll = 1;
constexpr int back_left__cluster_1_pitch = 2;
constexpr int back_left__cluster_2_roll = 3;
constexpr int back_left__cluster_2_pitch = 4;
constexpr int back_left__cluster_3_roll = 5;
constexpr int back_left__cluster_3_pitch = 6;
constexpr int back_left__cluster_3_wrist = 7;
constexpr int back_right__cluster_1_roll = 8;
constexpr int back_right__cluster_1_pitch = 9;
constexpr int back_right__cluster_2_roll = 10;
constexpr int back_right__cluster_2_pitch = 11;
constexpr int back_right__cluster_3_roll = 12;
constexpr int back_right__cluster_3_pitch = 13;
constexpr int back_right__cluster_3_wrist = 14;
constexpr int front_left__cluster_1_roll = 15;
constexpr int front_left__cluster_1_pitch = 16;
constexpr int front_left__cluster_2_roll = 17;
constexpr int front_left__cluster_2_pitch = 18;
constexpr int front_left__cluster_3_roll = 19;
constexpr int front_left__cluster_3_pitch = 20;
constexpr int front_left__cluster_3_wrist = 21;
constexpr int front_right__cluster_1_roll = 22;
constexpr int front_right__cluster_1_pitch = 23;
constexpr int front_right__cluster_2_roll = 24;
constexpr int front_right__cluster_2_pitch = 25;
constexpr int front_right__cluster_3_roll = 26;
constexpr int front_right__cluster_3_pitch = 27;
constexpr int front_right__cluster_3_wrist = 28;
} // namespace crab_joint

namespace crab {
constexpr int n_qdot = 18;
constexpr int n_adof = 28;
} // namespace crab
