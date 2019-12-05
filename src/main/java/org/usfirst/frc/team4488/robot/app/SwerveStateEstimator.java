package org.usfirst.frc.team4488.robot.app;

import org.usfirst.frc.team4488.lib.util.NavX;
import org.usfirst.frc.team4488.lib.util.app.math.Translation2d;
import org.usfirst.frc.team4488.lib.util.app.math.Twist2d;
import org.usfirst.frc.team4488.robot.systems.drive.SwerveDrive;
import org.usfirst.frc.team4488.robot.systems.drive.SwerveModule;

public class SwerveStateEstimator extends RobotStateEstimatorBase {

  private SwerveDrive swerve;
  private SwerveModule[] modules;
  private double[] prevDistances = new double[4];
  private double prevRobotTheta;
  private int numModules;

  public SwerveStateEstimator(SwerveDrive swerve) {
    this.swerve = swerve;
    modules = swerve.getModules();
    numModules = swerve.numModules();
  }

  @Override
  public void start() {
    for (int i = 0; i < numModules; i++) {
      prevDistances[i] = modules[i].getDistanceDriven();
    }
    prevRobotTheta = NavX.getInstance().getYaw().getRadians();
  }

  @Override
  public Twist2d generateOdometryFromSensors() {
    Translation2d[] moduleChanges = new Translation2d[numModules];
    for (int i = 0; i < numModules; i++) {
      double dist = modules[i].getDistanceDriven();
      double deltaDist = dist - prevDistances[i];
      prevDistances[i] = dist;
      double theta = modules[i].getFieldAngle();
      moduleChanges[i] = SwerveKinematics.forwardModuleKinematics(deltaDist, theta);
    }

    double theta = NavX.getInstance().getYaw().getRadians();
    double dTheta = theta - getLatestFieldToVehicle().getValue().getRotation().getRadians();
    Twist2d robotDelta = SwerveKinematics.forwardRobotKinematics(moduleChanges, dTheta);
    prevRobotTheta = theta;
    return robotDelta;
  }

  @Override
  public void addObservations(
      double timestamp, Twist2d measured_velocity, Twist2d predicted_velocity) {
    addFieldToVehicleObservation(
        timestamp,
        SwerveKinematics.integrateForwardKinematics(
            getLatestFieldToVehicle().getValue(), measured_velocity));
    vehicle_velocity_measured_ = measured_velocity;
    vehicle_velocity_predicted_ = predicted_velocity;
  }

  @Override
  public Twist2d generatePredictedVelocity() {
    return Twist2d.identity();
  }
}
