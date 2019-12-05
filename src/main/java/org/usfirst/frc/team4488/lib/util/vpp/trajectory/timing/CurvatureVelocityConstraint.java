package org.usfirst.frc.team4488.lib.util.vpp.trajectory.timing;

import org.usfirst.frc.team4488.lib.util.app.math.Pose2dWithCurvature;
import org.usfirst.frc.team4488.robot.Constants;

public class CurvatureVelocityConstraint implements TimingConstraint<Pose2dWithCurvature> {

  @Override
  public double getMaxVelocity(final Pose2dWithCurvature state) {
    return Constants.swerveTopSpeed / (1 + Math.abs(4.0 * state.getCurvature())); // 6.0
  }

  @Override
  public MinMaxAcceleration getMinMaxAcceleration(
      final Pose2dWithCurvature state, final double velocity) {
    return MinMaxAcceleration.kNoLimits;
  }
}
