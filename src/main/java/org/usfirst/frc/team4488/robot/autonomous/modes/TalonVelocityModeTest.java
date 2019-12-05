package org.usfirst.frc.team4488.robot.autonomous.modes;

import org.usfirst.frc.team4488.robot.autonomous.AutoModeBase;
import org.usfirst.frc.team4488.robot.autonomous.AutoModeEndedException;
import org.usfirst.frc.team4488.robot.systems.drive.Drive;

public class TalonVelocityModeTest extends AutoModeBase {

  @Override
  protected void routine() throws AutoModeEndedException {
    Drive mDrive = Drive.getInstance();
    mDrive.configureTalonsForSpeedControl();
    mDrive.setVelocitySetpoint(100, 100);
  }
}
