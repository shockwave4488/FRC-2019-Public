package org.usfirst.frc.team4488.lib.util.vpp.spline;

import org.usfirst.frc.team4488.lib.util.app.math.Pose2d;

public enum ReferenceFrame {
  NONE(new Pose2d()), // TODO: figure out these values
  START(new Pose2d()), // TODO: figure out these values
  SCALE(new Pose2d()); // TODO: figure out these values

  private Pose2d mReferenceFrame;

  ReferenceFrame(Pose2d mReferenceFrame) {
    this.mReferenceFrame = mReferenceFrame;
  }

  public Pose2d getReferenceFrame() {
    return this.mReferenceFrame;
  }
}
