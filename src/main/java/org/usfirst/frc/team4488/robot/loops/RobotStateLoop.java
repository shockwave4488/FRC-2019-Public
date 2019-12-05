package org.usfirst.frc.team4488.robot.loops;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4488.lib.util.app.math.RigidTransform2d;
import org.usfirst.frc.team4488.lib.util.app.math.Rotation2d;
import org.usfirst.frc.team4488.lib.util.app.math.Translation2d;
import org.usfirst.frc.team4488.lib.util.app.math.Twist2d;
import org.usfirst.frc.team4488.robot.app.RobotStateEstimatorBase;

/**
 * Periodically estimates the state of the robot using the robot's distance traveled (compares two
 * waypoints), gyroscope orientation, and velocity, among various other factors. Similar to a car's
 * odometer.
 */
public class RobotStateLoop implements Loop {
  private static RobotStateLoop instance_;

  public static RobotStateLoop createInstance(RobotStateEstimatorBase estimator) {
    instance_ = new RobotStateLoop(estimator);
    return instance_;
  }

  public static RobotStateLoop getInstance() {
    return instance_;
  }

  private RobotStateEstimatorBase estimator;

  private RobotStateLoop(RobotStateEstimatorBase estimator) {
    this.estimator = estimator;
    estimator.reset(0, new RigidTransform2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));
  }

  @Override
  public synchronized void onStart(double timestamp) {
    estimator.reset(0, new RigidTransform2d());
    estimator.start();
  }

  @Override
  public synchronized void onLoop(double timestamp) {
    final Twist2d odometry_velocity = estimator.generateOdometryFromSensors();
    final Twist2d predicted_velocity = estimator.generatePredictedVelocity();
    estimator.addObservations(timestamp, odometry_velocity, predicted_velocity);
    updateSmartDashboard();
  }

  @Override
  public void onStop(double timestamp) {
    // no-op
  }

  public void updateSmartDashboard() {
    RigidTransform2d pose = estimator.getLatestFieldToVehicle().getValue();
    Translation2d trans = pose.getTranslation();

    SmartDashboard.putNumber("pose x", trans.x());
    SmartDashboard.putNumber("pose y", trans.y());
    SmartDashboard.putNumber("pose theta", pose.getRotation().getDegrees());
  }

  public RobotStateEstimatorBase getEstimator() {
    return estimator;
  }
}
