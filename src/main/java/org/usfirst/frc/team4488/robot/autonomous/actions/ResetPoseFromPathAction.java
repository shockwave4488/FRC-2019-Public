package org.usfirst.frc.team4488.robot.autonomous.actions;

import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team4488.lib.util.app.math.RigidTransform2d;
import org.usfirst.frc.team4488.robot.app.RobotState;
import org.usfirst.frc.team4488.robot.app.paths.PathContainer;
import org.usfirst.frc.team4488.robot.systems.Drive;

/**
 * Resets the robot's current pose based on the starting pose stored in the pathContainer object.
 *
 * @see PathContainer
 * @see Action
 * @see RunOnceAction
 */
public class ResetPoseFromPathAction extends RunOnceAction {

  protected PathContainer mPathContainer;

  public ResetPoseFromPathAction(PathContainer pathContainer) {
    mPathContainer = pathContainer;
  }

  @Override
  public synchronized void runOnce() {
    RigidTransform2d startPose = mPathContainer.getStartPose();
    RobotState.getInstance().reset(Timer.getFPGATimestamp(), startPose);
    Drive.getInstance().setGyroAngle(startPose.getRotation());
  }
}
