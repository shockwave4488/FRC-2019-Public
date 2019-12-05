package JavaRoboticsLib.Drive.Interfaces;

import static JavaRoboticsLib.Utility.Util.accurateWaitSeconds;

import JavaRoboticsLib.ControlSystems.MotionController;
import JavaRoboticsLib.Drive.DriveEncoders;
import edu.wpi.first.wpilibj.Encoder;

public interface EncoderDrive extends TankDrive {

  public interface DynamicDriveDelegate {
    boolean drive(double valueLeft, double valueRight);
  }

  public interface DynamicEncoderDelegate {
    boolean drive(Encoder left, Encoder right);
  }

  public interface DynamicDriveEncoderDelegate {
    boolean drive(DriveEncoders encoders);
  }

  public DriveEncoders getEncoders();

  public default void driveToDistance(double location, double power, boolean brake, double interval)
      throws InterruptedException {
    double direction = getEncoders().getLinearDistance() < location ? -1 : 1;

    while ((getEncoders().getLinearDistance() - location) * direction > 0) {
      setPowers(power * direction, power * direction);
      accurateWaitSeconds(interval);
    }

    if (brake) setPowers(0, 0);
  }

  public default void driveToDistance(double location, double power, boolean brake)
      throws InterruptedException {
    driveToDistance(location, power, brake, 0.02);
  }

  public default void driveForDistance(
      double location, double power, boolean brake, double interval) throws InterruptedException {
    driveToDistance(location + getEncoders().getLinearDistance(), power, brake, interval);
  }

  public default void driveForDistance(double location, double power, boolean brake)
      throws InterruptedException {
    driveForDistance(location, power, brake, 0.02);
  }

  public default void driveToDistance(
      MotionController controller,
      double location,
      double tolerance,
      boolean brake,
      double interval)
      throws InterruptedException {
    controller.setSetPoint(location);

    while (Math.abs(location - getEncoders().getLinearDistance()) > tolerance) {
      double power = controller.get(getEncoders().getLinearDistance());
      setPowers(power, power);
      accurateWaitSeconds(interval);
    }

    if (brake) setPowers(0, 0);
  }

  public default void driveForDistance(
      MotionController controller,
      double location,
      double tolerance,
      boolean brake,
      double interval)
      throws InterruptedException {
    driveToDistance(
        controller, location + getEncoders().getLinearDistance(), tolerance, brake, interval);
  }

  public default void driveToDistance(
      MotionController controller, double location, double tolerance, boolean brake)
      throws InterruptedException {
    driveToDistance(controller, location, tolerance, brake, 0.02);
  }

  public default void driveForDistance(
      MotionController controller, double location, double tolerance, boolean brake)
      throws InterruptedException {
    driveForDistance(controller, location, tolerance, brake, 0.02);
  }

  public default void driveToAtSpeed(
      MotionController speedController,
      double speed,
      double location,
      boolean brake,
      double interval)
      throws InterruptedException {
    speedController.setSetPoint(getEncoders().getLinearDistance() < location ? speed : -speed);

    while ((getEncoders().getLinearDistance() - location)
            * Math.signum(speedController.getSetPoint())
        > 0) {
      double power = speedController.get(getEncoders().getLinearSpeed());
      setPowers(power, power);
      accurateWaitSeconds(interval);
    }

    if (brake) setPowers(0, 0);
  }

  public default void driveForAtSpeed(
      MotionController speedController,
      double speed,
      double location,
      boolean brake,
      double interval)
      throws InterruptedException {
    driveToAtSpeed(
        speedController, speed, location + getEncoders().getLinearDistance(), brake, interval);
  }

  public default void driveToAtSpeed(
      MotionController speedController, double speed, double location, boolean brake)
      throws InterruptedException {
    driveToAtSpeed(speedController, speed, location, brake, 0.02);
  }

  public default void driveForAtSpeed(
      MotionController speedController, double speed, double location, boolean brake)
      throws InterruptedException {
    driveForAtSpeed(speedController, speed, location, brake, 0.02);
  }

  public default void dynamicSpeedDrive(DynamicDriveDelegate expression, double interval)
      throws InterruptedException {
    while (!expression.drive(getEncoders().getLeftSpeed(), getEncoders().getRightSpeed())) {
      accurateWaitSeconds(interval);
    }
  }

  public default void dynamicSpeedDrive(DynamicDriveDelegate expression)
      throws InterruptedException {
    dynamicSpeedDrive(expression, 0.02);
  }

  public default void dynamicDistanceDrive(DynamicDriveDelegate expression, double interval)
      throws InterruptedException {
    while (!expression.drive(getEncoders().getLeftDistance(), getEncoders().getRightDistance()))
      accurateWaitSeconds(interval);
  }

  public default void dynamicDistanceDrive(DynamicDriveDelegate expression)
      throws InterruptedException {
    dynamicDistanceDrive(expression, 0.02);
  }

  public default void dynamicEncoderDrive(DynamicEncoderDelegate expression, double interval)
      throws InterruptedException {
    while (!expression.drive(getEncoders().getLeft(), getEncoders().getRight()))
      accurateWaitSeconds(interval);
  }

  public default void dynamicEncoderDrive(DynamicEncoderDelegate expression)
      throws InterruptedException {
    dynamicEncoderDrive(expression, 0.02);
  }

  public default void dynamicEncoderDrive(DynamicDriveEncoderDelegate expression, double interval)
      throws InterruptedException {
    while (!expression.drive(getEncoders())) {
      accurateWaitSeconds(interval);
    }
  }

  public default void dynamicEncoderDrive(DynamicDriveEncoderDelegate expression)
      throws InterruptedException {
    dynamicEncoderDrive(expression, 0.02);
  }
}
