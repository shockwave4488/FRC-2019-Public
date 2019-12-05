package org.usfirst.frc.team4488.lib.util.vpp.trajectory;

import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.usfirst.frc.team4488.lib.util.app.math.Pose2d;
import org.usfirst.frc.team4488.lib.util.app.math.Pose2dWithCurvature;
import org.usfirst.frc.team4488.lib.util.app.math.Rotation2d;
import org.usfirst.frc.team4488.lib.util.app.math.Translation2d;
import org.usfirst.frc.team4488.lib.util.vpp.DriveMotionPlanner;
import org.usfirst.frc.team4488.lib.util.vpp.trajectory.timing.TimedState;
import org.usfirst.frc.team4488.lib.util.vpp.trajectory.timing.TimingConstraint;
import org.usfirst.frc.team4488.robot.Constants;

public class TrajectoryGenerator {
  private static final double kMaxVelocity = 120.0;
  private static final double kMaxAccel = 120.0;
  private static final double kMaxDecel = 72.0;
  private static final double kMaxVoltage = 9.0;

  private static TrajectoryGenerator mInstance = new TrajectoryGenerator();
  private final DriveMotionPlanner mMotionPlanner;
  private TrajectorySet mTrajectorySet = null;

  public static TrajectoryGenerator getInstance() {
    return mInstance;
  }

  private TrajectoryGenerator() {
    mMotionPlanner = new DriveMotionPlanner();
  }

  public void generateTrajectories() {
    if (mTrajectorySet == null) {
      double startTime = Timer.getFPGATimestamp();
      System.out.println("Generating trajectories...");
      mTrajectorySet = new TrajectorySet();
      System.out.println(
          "Finished trajectory generation in: "
              + (Timer.getFPGATimestamp() - startTime)
              + " seconds");
    }
  }

  public TrajectorySet getTrajectorySet() {
    return mTrajectorySet;
  }

  public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
      boolean reversed,
      final List<Pose2d> waypoints,
      final List<TimingConstraint<Pose2dWithCurvature>> constraints,
      double max_vel, // inches/s
      double max_accel, // inches/s^2
      double max_decel,
      double max_voltage,
      double default_vel,
      int slowdown_chunks) {
    return mMotionPlanner.generateTrajectory(
        reversed,
        waypoints,
        constraints,
        max_vel,
        max_accel,
        max_decel,
        max_voltage,
        default_vel,
        slowdown_chunks);
  }

  public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
      boolean reversed,
      final List<Pose2d> waypoints,
      final List<TimingConstraint<Pose2dWithCurvature>> constraints,
      double start_vel, // inches/s
      double end_vel, // inches/s
      double max_vel, // inches/s
      double max_accel, // inches/s^2
      double max_decel,
      double max_voltage,
      double default_vel,
      int slowdown_chunks) {
    return mMotionPlanner.generateTrajectory(
        reversed,
        waypoints,
        constraints,
        start_vel,
        end_vel,
        max_vel,
        max_accel,
        max_decel,
        max_voltage,
        default_vel,
        slowdown_chunks);
  }

  // CRITICAL POSES
  // Origin is the center of the robot when the robot is placed against the middle of the alliance
  // station wall.
  // +x is towards the center of the field.
  // +y is to the right.
  // ALL POSES DEFINED FOR THE CASE THAT ROBOT STARTS ON LEFT! (mirrored about +x axis for RIGHT)
  /*
  Example from 1323
  static final Pose2d autoStartingPose = new Pose2d(Constants.kRobotLeftStartingPose.getTranslation().translateBy(new Translation2d(/*-0.50.0, 0.0)), Rotation2d.fromDegrees(-90.0));
  */

  public class TrajectorySet {
    public class MirroredTrajectory {
      public MirroredTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> left) {
        this.left = left;
        this.right = TrajectoryUtil.mirrorTimed(left, left.defaultVelocity());
      }

      public Trajectory<TimedState<Pose2dWithCurvature>> get(boolean left) {
        return left ? this.left : this.right;
      }

      public final Trajectory<TimedState<Pose2dWithCurvature>> left;
      public final Trajectory<TimedState<Pose2dWithCurvature>> right;
    }

    // Test Paths
    public final Trajectory<TimedState<Pose2dWithCurvature>> straightPath;
    public final Trajectory<TimedState<Pose2dWithCurvature>> secondDemoPath;
    public final Trajectory<TimedState<Pose2dWithCurvature>> thirdDemoPath;

    private TrajectorySet() {
      // Test Paths
      straightPath = getStraightPath();
      secondDemoPath = getSecondDemoPath();
      thirdDemoPath = getThirdDemoPath();
    }

    private Trajectory<TimedState<Pose2dWithCurvature>> getStraightPath() {
      List<Pose2d> waypoints = new ArrayList<>();
      waypoints.add(Constants.kRobotLeftStartingPose);
      waypoints.add(new Pose2d(new Translation2d(142, 10), Rotation2d.fromRadians(0)));
      waypoints.add(new Pose2d(new Translation2d(142, -79), Rotation2d.fromRadians(0)));
      waypoints.add(new Pose2d(new Translation2d(10, -79), Rotation2d.fromRadians(0)));
      return generateTrajectory(
          false,
          waypoints,
          Arrays.asList(),
          kMaxVelocity,
          kMaxAccel,
          kMaxDecel,
          kMaxVoltage,
          60.0,
          1);
    }

    private Trajectory<TimedState<Pose2dWithCurvature>> getSecondDemoPath() {
      List<Pose2d> waypoints = new ArrayList<>();
      waypoints.add(new Pose2d(new Translation2d(10, -79), Rotation2d.fromRadians(0)));
      waypoints.add(new Pose2d(new Translation2d(274, -79), Rotation2d.fromRadians(0)));
      return generateTrajectory(
          false,
          waypoints,
          Arrays.asList(),
          kMaxVelocity,
          kMaxAccel,
          kMaxDecel,
          kMaxVoltage,
          60,
          1);
    }

    private Trajectory<TimedState<Pose2dWithCurvature>> getThirdDemoPath() {
      List<Pose2d> waypoints = new ArrayList<>();
      waypoints.add(new Pose2d(new Translation2d(274, -79), Rotation2d.fromRadians(0)));
      waypoints.add(new Pose2d(new Translation2d(136, -85), Rotation2d.fromRadians(0)));
      waypoints.add(new Pose2d(new Translation2d(139, 108), Rotation2d.fromRadians(0)));
      waypoints.add(new Pose2d(new Translation2d(10, 105), Rotation2d.fromRadians(0)));
      return generateTrajectory(
          false,
          waypoints,
          Arrays.asList(),
          kMaxVelocity,
          kMaxAccel,
          kMaxDecel,
          kMaxVoltage,
          60,
          1);
    }
  }
}
