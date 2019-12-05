package org.usfirst.frc.team4488.robot.autonomous.actions;

import org.usfirst.frc.team4488.robot.operator.Logging;
import org.usfirst.frc.team4488.robot.systems.Drive;
import org.usfirst.frc.team4488.robot.systems.SmartDrive;

public class DirectDriveStraightAction implements Action {
  Logging logger = Logging.getInstance();

  private double desiredDistance;
  private double startDistance;
  private double startAngle;
  private int updateCycles = 0;

  SmartDrive smartDrive;

  public DirectDriveStraightAction(double distance) {
    smartDrive = new SmartDrive(Drive.getInstance());
    desiredDistance = distance;
  }

  @Override
  public boolean isFinished() {
    // TODO Auto-generated method stub
    return smartDrive.isDriveDistanceDone() && updateCycles > 3;
  }

  @Override
  public void update() {
    // TODO Auto-generated method stub
    if (!isFinished()) {
      // @ TODO Fix angle
      smartDrive.driveToDistance(desiredDistance + startDistance, startAngle);
      updateCycles++;
    }
  }

  @Override
  public void done() {
    // TODO Auto-generated method stub
    smartDrive.stop();
    logger.writeToLogFormatted(this, "done()");
  }

  @Override
  public void start() {
    logger.writeToLogFormatted(this, "start()");
    startAngle = Drive.getInstance().getGyroscope().getYaw();
    startDistance = Drive.getInstance().getLinearDistance();
    // TODO Auto-generated method stub

  }
}
