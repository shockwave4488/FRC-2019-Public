package org.usfirst.frc.team4488.robot.systems.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4488.lib.util.NavX;
import org.usfirst.frc.team4488.lib.util.app.Util;
import org.usfirst.frc.team4488.lib.util.app.math.Pose2d;
import org.usfirst.frc.team4488.lib.util.app.math.Pose2dWithCurvature;
import org.usfirst.frc.team4488.lib.util.app.math.RigidTransform2d;
import org.usfirst.frc.team4488.lib.util.app.math.Translation2d;
import org.usfirst.frc.team4488.lib.util.vpp.DriveMotionPlanner;
import org.usfirst.frc.team4488.lib.util.vpp.SwerveHeadingController;
import org.usfirst.frc.team4488.lib.util.vpp.trajectory.TimedView;
import org.usfirst.frc.team4488.lib.util.vpp.trajectory.Trajectory;
import org.usfirst.frc.team4488.lib.util.vpp.trajectory.TrajectoryIterator;
import org.usfirst.frc.team4488.lib.util.vpp.trajectory.timing.TimedState;
import org.usfirst.frc.team4488.robot.Constants;
import org.usfirst.frc.team4488.robot.RobotMap;
import org.usfirst.frc.team4488.robot.app.RobotStateEstimatorBase;
import org.usfirst.frc.team4488.robot.app.SwerveStateEstimator;
import org.usfirst.frc.team4488.robot.loops.Loop;
import org.usfirst.frc.team4488.robot.loops.Looper;
import org.usfirst.frc.team4488.robot.loops.RobotStateLoop;

public class SwerveDrive extends DriveBase {

  private SwerveModule[] modules;

  private final double radius =
      Math.hypot(Constants.kTrackLengthInches, Constants.kTrackWidthInches);
  private final double length = Constants.kTrackLengthInches / radius;
  private final double width = Constants.kTrackWidthInches / radius;

  private static SwerveDrive sInstance;

  private DriveMotionPlanner motionPlanner = new DriveMotionPlanner();
  private SwerveHeadingController headingController = new SwerveHeadingController();
  private SwerveStateEstimator stateEstimator = new SwerveStateEstimator(this);

  private final int numModules;

  private enum ControlState {
    PathFollowing,
    Manual;
  }

  private ControlState controlState = ControlState.Manual;

  private boolean hasStartedPath = false;
  private boolean hasFinishedPath = false;
  private double rotationScalar;
  private Translation2d lastTrajectoryVector = Translation2d.identity();

  public static SwerveDrive getInstance() {
    if (sInstance == null) {
      sInstance = new SwerveDrive();
    }

    return sInstance;
  }

  private Loop mLoop =
      new Loop() {
        @Override
        public void onStart(double timestamp) {
          RobotStateLoop.getInstance()
              .getEstimator()
              .reset(
                  0,
                  new RigidTransform2d(
                      Constants.kRobotLeftStartingPose.getTranslation(),
                      Constants.kRobotLeftStartingPose.getRotation()));
        }

        @Override
        public void onLoop(double timestamp) {
          switch (controlState) {
            case PathFollowing:
              updatePathFollower(timestamp);
              break;
            case Manual:
              break;
          }
        }

        @Override
        public void onStop(double timestamp) {}
      };

  public SwerveDrive() {
    numModules = RobotMap.swerveParameters.length;
    modules = new SwerveModule[numModules];
    for (int i = 0; i < numModules; i++) {
      modules[i] = new SwerveModule(RobotMap.swerveParameters[i]);
    }
  }

  @Override
  public void controllerUpdate(double leftStickX, double leftStickY, double rightStickX) {
    driveVoltage(leftStickY, leftStickX, rightStickX, true);
  }

  private Translation2d[] calcModuleVectors(
      double forward, double strafe, double rotation, double maxThrottle) {
    final double a = strafe - rotation * length;
    final double b = strafe + rotation * length;
    final double c = forward - rotation * width;
    final double d = forward + rotation * width;

    double[] speeds = new double[4];
    speeds[0] = Math.hypot(b, d);
    speeds[1] = Math.hypot(b, c);
    speeds[2] = Math.hypot(a, d);
    speeds[3] = Math.hypot(a, c);

    double[] angles = new double[4];
    angles[0] = Math.atan2(b, d);
    angles[1] = Math.atan2(b, c);
    angles[2] = Math.atan2(a, d);
    angles[3] = Math.atan2(a, c);

    // clamp speeds to max throttle
    final double maxSpeed =
        Math.max(Math.max(speeds[0], speeds[1]), Math.max(speeds[2], speeds[3]));
    if (maxSpeed > maxThrottle) {
      for (int i = 0; i < numModules; i++) {
        speeds[i] /= maxSpeed;
        speeds[i] *= maxThrottle;
      }
    }

    Translation2d[] vectors = new Translation2d[numModules];
    for (int i = 0; i < numModules; i++) {
      Translation2d vector =
          new Translation2d(Math.cos(angles[i]) * speeds[i], Math.sin(angles[i]) * speeds[i]);
      vectors[i] = vector;
    }

    return vectors;
  }

  public void driveVoltage(double forward, double strafe, double rotation, boolean fieldCentric) {

    if (fieldCentric) {
      double angle = (NavX.getInstance().getAHRS().getAngle() - 180) % 360;
      angle = Math.toRadians(angle);

      double newForward = forward * Math.cos(angle) + strafe * Math.sin(angle);
      strafe = strafe * Math.cos(angle) - forward * Math.sin(angle);
      forward = newForward;
    }

    // Anything less that 0.01 doesnt matter
    if (Util.epsilonEquals(forward, 0, 0.01)
        && Util.epsilonEquals(strafe, 0, 0.01)
        && Util.epsilonEquals(rotation, 0, 0.01)) {
      for (SwerveModule module : modules) {
        module.stop();
      }
      return;
    }

    Translation2d[] vectors = calcModuleVectors(forward, strafe, rotation, 1);

    for (int i = 0; i < numModules; i++) {
      modules[i].driveVoltage(vectors[i]);
    }
  }

  public void driveVelocityComponents(
      double forwardVel, double strafeVel, double rotation, boolean fieldCentric) {

    if (fieldCentric) {
      double angle = (NavX.getInstance().getAHRS().getAngle() - 180) % 360;
      angle = Math.toRadians(angle);

      double newForward = forwardVel * Math.cos(angle) + strafeVel * Math.sin(angle);
      strafeVel = strafeVel * Math.cos(angle) - forwardVel * Math.sin(angle);
      forwardVel = newForward;
    }

    // Anything less than 0.01 doesnt matter
    if (Util.epsilonEquals(forwardVel, 0, 0.01)
        && Util.epsilonEquals(strafeVel, 0, 0.01)
        && Util.epsilonEquals(rotation, 0, 0.01)) {
      for (SwerveModule module : modules) {
        module.stop();
      }
      return;
    }

    forwardVel /= Constants.swerveTopSpeed;
    strafeVel /= Constants.swerveTopSpeed;

    Translation2d[] vectors = calcModuleVectors(forwardVel, strafeVel, rotation, 1);

    for (int i = 0; i < numModules; i++) {
      modules[i].driveVelocity(vectors[i].scale(Constants.swerveTopSpeed));
    }
  }

  public void driveVelocityVector(
      double velocity, double theta, double rotation, boolean fieldCentric) {
    double forwardVel = Math.cos(theta) * velocity;
    double strafeVel = Math.sin(theta) * velocity;
    driveVelocityComponents(forwardVel, strafeVel, rotation, fieldCentric);
  }

  public void driveVelocityVector(Translation2d vector, double rotation, boolean fieldCentric) {
    double velocity = vector.norm();
    double theta = vector.direction().getRadians();
    driveVelocityVector(velocity, theta, rotation, fieldCentric);
  }

  public void stop() {
    for (SwerveModule module : modules) {
      module.driveVoltage(Translation2d.identity());
    }
  }

  public SwerveModule[] getModules() {
    return modules;
  }

  public int numModules() {
    return numModules;
  }

  private void updatePathFollower(double timestamp) {
    if (!motionPlanner.isDone()) {
      Pose2d pose =
          RobotStateLoop.getInstance()
              .getEstimator()
              .getLatestFieldToVehicle()
              .getValue()
              .toPose2d();
      double rotationCorrection =
          headingController.updateRotationCorrection(pose.getRotation().getDegrees(), timestamp);
      Translation2d driveVector =
          motionPlanner.update(
              timestamp,
              RobotStateLoop.getInstance()
                  .getEstimator()
                  .getLatestFieldToVehicle()
                  .getValue()
                  .toPose2d());

      double rotationInput =
          Util.deadBand(
              Util.limit(
                  rotationCorrection * rotationScalar * driveVector.norm() * -1,
                  motionPlanner.getMaxRotationSpeed()),
              0.01);
      if (Util.epsilonEquals(driveVector.norm(), 0.0, 0.0001)) {
        driveVector = lastTrajectoryVector;
      } else {
      }

      driveVelocityVector(driveVector.inverse(), rotationInput, true);

      lastTrajectoryVector = driveVector;
    } else {
      hasFinishedPath = true;
      driveVelocityVector(0, 0, 0, true);
    }
  }

  public void setTrajectory(
      Trajectory<TimedState<Pose2dWithCurvature>> trajectory,
      double targetHeading,
      double rotationScalar,
      Translation2d followingCenter) {
    hasStartedPath = false;
    hasFinishedPath = false;
    this.rotationScalar = rotationScalar;

    motionPlanner.reset();
    motionPlanner.setTrajectory(new TrajectoryIterator<>(new TimedView<>(trajectory)));
    motionPlanner.setFollowingCenter(followingCenter);

    headingController.setSnapTarget(targetHeading);

    controlState = ControlState.PathFollowing;
  }

  public void setTrajectory(
      Trajectory<TimedState<Pose2dWithCurvature>> trajectory,
      double targetHeading,
      double rotationScalar) {
    setTrajectory(trajectory, targetHeading, rotationScalar, Translation2d.identity());
  }

  public boolean isDoneWithPath() {
    return hasFinishedPath;
  }

  @Override
  public void zeroSensors() {
    NavX.getInstance().zeroYaw();
  }

  @Override
  public void reset() {}

  @Override
  public void updateSmartDashboard() {
    SmartDashboard.putNumber("gyro", NavX.getInstance().getYaw().getDegrees());
  }

  @Override
  public void registerEnabledLoops(Looper looper) {
    for (SwerveModule module : modules) {
      looper.register(module);
    }

    looper.register(mLoop);
  }

  @Override
  public void updatePrefs() {
    for (SwerveModule module : modules) {
      module.updatePrefs();
    }
  };

  @Override
  public void writeToLog() {};

  @Override
  public RobotStateEstimatorBase getStateEstimator() {
    return stateEstimator;
  }

  @Override
  public void configPercentVbus() {}

  @Override
  public void resetAngle() {
    NavX.getInstance().reset();
  }
}
