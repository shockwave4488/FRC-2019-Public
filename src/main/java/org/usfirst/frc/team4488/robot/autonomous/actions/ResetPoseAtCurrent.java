package org.usfirst.frc.team4488.robot.autonomous.actions;

import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team4488.lib.util.app.math.RigidTransform2d;
import org.usfirst.frc.team4488.lib.util.app.math.Rotation2d;
import org.usfirst.frc.team4488.lib.util.app.math.Translation2d;
import org.usfirst.frc.team4488.robot.app.RobotState;

public class ResetPoseAtCurrent extends RunOnceAction {

  @Override
  public void runOnce() {
    RobotState.getInstance()
        .reset(
            Timer.getFPGATimestamp(),
            new RigidTransform2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));
  }
}
