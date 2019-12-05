package org.usfirst.frc.team4488.robot.autonomous.actions;

import org.usfirst.frc.team4488.robot.systems.drive.Drive;

public class DriveStraightDistance implements Action {

  private Drive drive = Drive.getInstance();
  private double distance;
  private double throttle;
  private double startDistance;

  public DriveStraightDistance(double distance, double throttle) {
    this.distance = distance;
    this.throttle = throttle;
  }

  @Override
  public void start() {
    startDistance = drive.getLinearDistance();
    drive.driveStraightField(throttle);
  }

  @Override
  public void update() {
    drive.driveStraightField(throttle);
  }

  @Override
  public boolean isFinished() {
    return drive.getLinearDistance() - startDistance > distance;
  }

  @Override
  public void done() {
    drive.setPowers(0, 0);
  }
}
