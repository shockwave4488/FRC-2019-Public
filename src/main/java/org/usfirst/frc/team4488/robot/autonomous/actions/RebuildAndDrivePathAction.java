package org.usfirst.frc.team4488.robot.autonomous.actions;

import org.usfirst.frc.team4488.lib.util.app.control.Path;
import org.usfirst.frc.team4488.robot.app.paths.PathContainer;
import org.usfirst.frc.team4488.robot.operator.Logging;
import org.usfirst.frc.team4488.robot.systems.drive.Drive;

/**
 * Drives the robot along the Path defined in the PathContainer object. The action finishes once the
 * robot reaches the end of the path.
 *
 * @see PathContainer
 * @see Path
 * @see Action
 */
public class RebuildAndDrivePathAction implements Action {
  Logging logger = Logging.getInstance();

  private PathContainer mPathContainer;
  private Path mPath;
  private Drive mDrive = Drive.getInstance();

  public RebuildAndDrivePathAction(PathContainer p) {
    mPathContainer = p;
  }

  @Override
  public boolean isFinished() {
    return mDrive.isDoneWithPath();
  }

  @Override
  public void update() {
    // Nothing done here, controller updates in mEnabedLooper in robot
  }

  @Override
  public void done() {
    // TODO: Perhaps set wheel velocity to 0?
    logger.writeToLogFormatted(this, "done()");
  }

  @Override
  public void start() {
    logger.writeToLogFormatted(this, "start()");
    mPath = mPathContainer.buildPath();
    mDrive.setWantDrivePath(mPath, mPathContainer.isReversed());
  }
}
