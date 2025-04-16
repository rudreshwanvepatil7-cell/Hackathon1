#include "mujoco_utils.hpp"

namespace mjc_utils{

    void SetJointAndActuatorMaps(
        mjModel *m, 
        std::unordered_map<std::string, int> &mj_qpos_map,
        std::unordered_map<std::string, int> &mj_qvel_map,
        std::unordered_map<std::string, int> &mj_actuator_map,
        const std::unordered_map<std::string, int> &pin_joint_map,
        const std::unordered_map<std::string, int> &pin_actuator_map) 
    {
        // create mujoco joint map
        for (const auto &[pin_jnt_name, _] : pin_joint_map) {
            int jnt_idx = mj_name2id(m, mjOBJ_JOINT, pin_jnt_name.c_str());
            if (jnt_idx == -1) {
                std::cout << "[MuJoCo Utils] Can't find the joint name: " 
                          << pin_jnt_name << " in MuJoCo model" << '\n';
            }
            mj_qpos_map[pin_jnt_name] = m->jnt_qposadr[jnt_idx];
            mj_qvel_map[pin_jnt_name] = m->jnt_dofadr[jnt_idx];
        }
        // create mujoco actuator map
        for (const auto &[pin_act_name, _] : pin_actuator_map) {
            int act_idx = mj_name2id(m, mjOBJ_ACTUATOR, pin_act_name.c_str());
            if (act_idx == -1) {
                std::cout << "[MuJoCo Utils] Can't find the actuator name: " 
                          << pin_act_name << " in MuJoCo model" << '\n';
            }
            mj_actuator_map[pin_act_name] = act_idx;
        }
    }
}
