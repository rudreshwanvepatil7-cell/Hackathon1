#pragma once

class CrabSensorData;
class PinocchioRobotSystem;

class StateEstimator {
public:
  StateEstimator(PinocchioRobotSystem *robot) : robot_(robot) {};
  virtual ~StateEstimator() = default;

  virtual void Initialize(CrabSensorData *sensor_Data) = 0;
  virtual void Update(CrabSensorData *sensor_Data) = 0;

  // simulation only
  virtual void UpdateGroundTruthSensorData(CrabSensorData *sensor_data) = 0;

protected:
  PinocchioRobotSystem *robot_;
};
