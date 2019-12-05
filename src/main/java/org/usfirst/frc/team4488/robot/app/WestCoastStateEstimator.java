package org.usfirst.frc.team4488.robot.app;

import org.usfirst.frc.team4488.lib.util.app.math.RigidTransform2d;
import org.usfirst.frc.team4488.lib.util.app.math.Rotation2d;
import org.usfirst.frc.team4488.lib.util.app.math.Twist2d;
import org.usfirst.frc.team4488.robot.systems.drive.Drive;

public class WestCoastStateEstimator extends RobotStateEstimatorBase {

  private Drive drive_;
  private double left_encoder_prev_distance_;
  private double right_encoder_prev_distance_;

  public WestCoastStateEstimator(Drive drive) {
    drive_ = drive;
  }

  @Override
  public void start() {
    left_encoder_prev_distance_ = drive_.getLeftDistance();
    right_encoder_prev_distance_ = drive_.getRightDistance();
  }

  @Override
  public Twist2d generateOdometryFromSensors() {
    final double left_distance = drive_.getLeftDistance();
    final double right_distance = drive_.getRightDistance();

    final double left_distance_delta = left_distance - left_encoder_prev_distance_;
    final double right_distance_delta = right_distance - right_encoder_prev_distance_;
    final Rotation2d gyro_angle = drive_.getAngleRotation2d();

    final RigidTransform2d last_measurement = getLatestFieldToVehicle().getValue();
    final Twist2d delta =
        WestCoastKinematics.forwardKinematics(
            last_measurement.getRotation(), left_distance_delta, right_distance_delta, gyro_angle);
    distanceDriven += delta.dx;

    left_encoder_prev_distance_ = left_distance;
    right_encoder_prev_distance_ = right_distance;
    return delta;
  }

  @Override
  public synchronized void addObservations(
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
    return WestCoastKinematics.forwardKinematics(drive_.getLeftSpeed(), drive_.getRightSpeed());
  }
}
