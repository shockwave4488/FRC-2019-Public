package org.usfirst.frc.team4488.robot.autonomous.actions;

import org.usfirst.frc.team4488.robot.operator.Logging;
import org.usfirst.frc.team4488.robot.systems.drive.Drive;
import org.usfirst.frc.team4488.robot.systems.drive.SmartDrive;

/**
 * Turns the robot to a specified heading
 *
 * @see Action
 */
public class TurnToHeadingAction implements Action {
  Logging logger = Logging.getInstance();

  private int mTargetHeading;
  private SmartDrive smartDrive;

  public TurnToHeadingAction(int heading) {
    mTargetHeading = heading;
    smartDrive = new SmartDrive(Drive.getInstance());
  }

  @Override
  public boolean isFinished() {
    return smartDrive.isTurnDone();
  }

  @Override
  public void update() {
    smartDrive.turnToAngle(mTargetHeading);
  }

  @Override
  public void done() {
    logger.writeToLogFormatted(this, "done()");
    smartDrive.stop();
  }

  @Override
  public void start() {
    Drive.getInstance().configPercentVbus();
    logger.writeToLogFormatted(this, "start()");
    smartDrive.turnToAngle(mTargetHeading);
  }
}
