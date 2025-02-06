#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "controller/crab_controller/crab_interface.hpp"
#include "controller/interrupt_handler.hpp"

class PyInterface : public Interface {
 public:
  using Interface::Interface;
  void GetCommand(void *sensor_data, void *command_data) override {
    PYBIND11_OVERRIDE_PURE(void, Interface, GetCommand, sensor_data,
                           command_data);
  }

 protected:
  void _SetParameters() override {}
};

namespace py = pybind11;

PYBIND11_MODULE(crab_interface_py, m) {
  py::module::import("interrupt_py");

  py::class_<Interface, PyInterface>(m, "Interface")
      .def(py::init<>())
      .def("GetCommand", &Interface::GetCommand);

  py::class_<CrabInterface, Interface>(m, "CrabInterface")
      .def(py::init<>())
      .def_readwrite("interrupt_", &CrabInterface::interrupt_handler_);

  py::class_<CrabSensorData>(m, "CrabSensorData")
      .def(py::init<>())
      .def_readwrite("imu_frame_quat_", &CrabSensorData::imu_frame_quat_)
      .def_readwrite("imu_ang_vel_", &CrabSensorData::imu_ang_vel_)
      .def_readwrite("joint_pos_", &CrabSensorData::joint_pos_)
      .def_readwrite("joint_vel_", &CrabSensorData::joint_vel_)
      .def_readwrite("b_FL_foot_contact_", &CrabSensorData::b_FL_foot_contact_)
      .def_readwrite("b_FR_foot_contact_", &CrabSensorData::b_FR_foot_contact_)
      .def_readwrite("b_RL_foot_contact_", &CrabSensorData::b_RL_foot_contact_)
      .def_readwrite("b_RR_foot_contact_", &CrabSensorData::b_RR_foot_contact_)
      .def_readwrite("FL_normal_force_", &CrabSensorData::FL_normal_force_)
      .def_readwrite("FR_normal_force_", &CrabSensorData::FR_normal_force_)
      .def_readwrite("RL_normal_force_", &CrabSensorData::RL_normal_force_)
      .def_readwrite("RR_normal_force_", &CrabSensorData::RR_normal_force_)
      .def_readwrite("lfoot_target_vector_", &CrabSensorData::lfoot_target_vector_) 
      .def_readwrite("rfoot_target_vector_", &CrabSensorData::rfoot_target_vector_) 
      .def_readwrite("lhand_target_vector_", &CrabSensorData::lhand_target_vector_) 
      .def_readwrite("rhand_target_vector_", &CrabSensorData::rhand_target_vector_) 
      .def_readwrite("imu_dvel_", &CrabSensorData::imu_dvel_)
      .def_readwrite("imu_lin_acc_", &CrabSensorData::imu_lin_acc_)

      // Debug
      .def_readwrite("base_joint_pos_", &CrabSensorData::base_joint_pos_)
      .def_readwrite("base_joint_quat_", &CrabSensorData::base_joint_quat_)
      .def_readwrite("base_joint_lin_vel_",
                     &CrabSensorData::base_joint_lin_vel_)
      .def_readwrite("base_joint_ang_vel_",
                     &CrabSensorData::base_joint_ang_vel_);

  py::class_<CrabCommand>(m, "CrabCommand")
      .def(py::init<>())
      .def_readwrite("joint_pos_cmd_", &CrabCommand::joint_pos_cmd_)
      .def_readwrite("joint_vel_cmd_", &CrabCommand::joint_vel_cmd_)
      .def_readwrite("joint_trq_cmd_", &CrabCommand::joint_trq_cmd_);
}
