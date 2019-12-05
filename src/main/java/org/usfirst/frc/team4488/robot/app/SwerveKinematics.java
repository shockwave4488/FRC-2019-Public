package org.usfirst.frc.team4488.robot.app;

import org.usfirst.frc.team4488.lib.util.app.math.RigidTransform2d;
import org.usfirst.frc.team4488.lib.util.app.math.Rotation2d;
import org.usfirst.frc.team4488.lib.util.app.math.Translation2d;
import org.usfirst.frc.team4488.lib.util.app.math.Twist2d;

public class SwerveKinematics {

  public static Translation2d forwardModuleKinematics(double deltaDistance, double theta) {
    return new Translation2d(
        Math.cos(theta) * deltaDistance, // dx
        Math.sin(theta) * deltaDistance // dy
        );
  }

  public static RigidTransform2d integrateForwardKinematics(
      RigidTransform2d currentPose, Twist2d forwardKinematics) {
    Translation2d forwardTranslation =
        new Translation2d(forwardKinematics.dx, forwardKinematics.dy);
    currentPose.setTranslation(currentPose.getTranslation().translateBy(forwardTranslation));
    Rotation2d forwardRotation = Rotation2d.fromRadians(forwardKinematics.dtheta);
    currentPose.setRotation(currentPose.getRotation().rotateBy(forwardRotation));

    return currentPose;
  }

  public static Twist2d forwardRobotKinematics(Translation2d[] moduleDeltas, double deltaTheta) {
    double avgDX = 0;
    double avgDY = 0;
    for (Translation2d moduleDelta : moduleDeltas) {
      avgDX += moduleDelta.x();
      avgDY += moduleDelta.y();
    }
    avgDX /= 4;
    avgDY /= 4;

    return new Twist2d(avgDX, avgDY, deltaTheta);
  }
}
