package org.usfirst.frc.team4488.robot.systems.drive;

import org.usfirst.frc.team4488.robot.app.RobotStateEstimatorBase;
import org.usfirst.frc.team4488.robot.systems.Subsystem;

public abstract class DriveBase extends Subsystem {
  public abstract void configPercentVbus();

  public abstract void resetAngle();

  public abstract RobotStateEstimatorBase getStateEstimator();

  public abstract void controllerUpdate(double leftStickX, double leftStickY, double rightStickX);
}
