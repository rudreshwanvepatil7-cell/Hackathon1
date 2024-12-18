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

  py::class_<crabInterface, Interface>(m, "crabInterface")
      .def(py::init<>())
      .def_readwrite("interrupt_", &crabInterface::interrupt_handler_);

  py::class_<crabSensorData>(m, "crabSensorData")
      .def(py::init<>())
      .def_readwrite("imu_frame_quat_", &crabSensorData::imu_frame_quat_)
      .def_readwrite("imu_ang_vel_", &crabSensorData::imu_ang_vel_)
      .def_readwrite("joint_pos_", &crabSensorData::joint_pos_)
      .def_readwrite("joint_vel_", &crabSensorData::joint_vel_)
      .def_readwrite("b_lf_contact_", &crabSensorData::b_lf_contact_)
      .def_readwrite("b_rf_contact_", &crabSensorData::b_rf_contact_)
      .def_readwrite("lf_contact_normal_", &crabSensorData::lf_contact_normal_)
      .def_readwrite("rf_contact_normal_", &crabSensorData::rf_contact_normal_)
      .def_readwrite("imu_dvel_", &crabSensorData::imu_dvel_)
      .def_readwrite("imu_lin_acc_", &crabSensorData::imu_lin_acc_)

      // Debug
      .def_readwrite("base_joint_pos_", &crabSensorData::base_joint_pos_)
      .def_readwrite("base_joint_quat_", &crabSensorData::base_joint_quat_)
      .def_readwrite("base_joint_lin_vel_",
                     &crabSensorData::base_joint_lin_vel_)
      .def_readwrite("base_joint_ang_vel_",
                     &crabSensorData::base_joint_ang_vel_);

  py::class_<crabCommand>(m, "crabCommand")
      .def(py::init<>())
      .def_readwrite("joint_pos_cmd_", &crabCommand::joint_pos_cmd_)
      .def_readwrite("joint_vel_cmd_", &crabCommand::joint_vel_cmd_)
      .def_readwrite("joint_trq_cmd_", &crabCommand::joint_trq_cmd_);
}
