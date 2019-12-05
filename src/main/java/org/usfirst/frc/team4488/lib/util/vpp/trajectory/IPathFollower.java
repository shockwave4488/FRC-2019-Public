package org.usfirst.frc.team4488.lib.util.vpp.trajectory;

import org.usfirst.frc.team4488.lib.util.app.math.Pose2d;
import org.usfirst.frc.team4488.lib.util.app.math.Twist2d;

public interface IPathFollower {
  public Twist2d steer(Pose2d current_pose);

  public boolean isDone();
}
