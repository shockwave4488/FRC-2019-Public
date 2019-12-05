package org.usfirst.frc.team4488.robot.loops;

import org.usfirst.frc.team4488.lib.util.app.math.Rotation2d;
import org.usfirst.frc.team4488.lib.util.app.math.Twist2d;
import org.usfirst.frc.team4488.robot.app.Kinematics;
import org.usfirst.frc.team4488.robot.app.RobotState;
import org.usfirst.frc.team4488.robot.systems.Drive;

/**
 * Periodically estimates the state of the robot using the robot's distance traveled (compares two
 * waypoints), gyroscope orientation, and velocity, among various other factors. Similar to a car's
 * odometer.
 */
public class RobotStateEstimator implements Loop {
  static RobotStateEstimator instance_ = new RobotStateEstimator();

  public static RobotStateEstimator getInstance() {
    return instance_;
  }

  RobotStateEstimator() {}

  RobotState robot_state_ = RobotState.getInstance();
  Drive drive_ = Drive.getInstance();
  double left_encoder_prev_distance_ = 0;
  double right_encoder_prev_distance_ = 0;

  @Override
  public synchronized void onStart(double timestamp) {
    left_encoder_prev_distance_ = drive_.getLeftDistance();
    right_encoder_prev_distance_ = drive_.getRightDistance();
  }

  @Override
  public synchronized void onLoop(double timestamp) {
    final double left_distance = drive_.getLeftDistance();
    final double right_distance = drive_.getRightDistance();
    final Rotation2d gyro_angle = drive_.getAngleRotation2d();
    final Twist2d odometry_velocity =
        robot_state_.generateOdometryFromSensors(
            left_distance - left_encoder_prev_distance_,
            right_distance - right_encoder_prev_distance_,
            gyro_angle);
    final Twist2d predicted_velocity =
        Kinematics.forwardKinematics(drive_.getLeftSpeed(), drive_.getRightSpeed());
    robot_state_.addObservations(timestamp, odometry_velocity, predicted_velocity);
    left_encoder_prev_distance_ = left_distance;
    right_encoder_prev_distance_ = right_distance;
  }

  @Override
  public void onStop(double timestamp) {
    // no-op
  }
}
