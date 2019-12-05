package org.usfirst.frc.team4488.robot.app;

import org.usfirst.frc.team4488.lib.util.app.math.Twist2d;

public class DummyStateEstimator extends RobotStateEstimatorBase {

  @Override
  public void addObservations(
      double timestamp, Twist2d measured_velocity, Twist2d predicted_velocity) {
    addFieldToVehicleObservation(
        timestamp,
        WestCoastKinematics.integrateForwardKinematics(
            getLatestFieldToVehicle().getValue(), measured_velocity));
    vehicle_velocity_measured_ = measured_velocity;
    vehicle_velocity_predicted_ = predicted_velocity;
  }

  @Override
  public Twist2d generatePredictedVelocity() {
    return Twist2d.identity();
  }

  @Override
  public Twist2d generateOdometryFromSensors() {
    return Twist2d.identity();
  }

  @Override
  public void start() {}
}
