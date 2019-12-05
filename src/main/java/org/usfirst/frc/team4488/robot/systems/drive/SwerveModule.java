package org.usfirst.frc.team4488.robot.systems.drive;

import JavaRoboticsLib.ControlSystems.SimPID;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import org.usfirst.frc.team4488.lib.util.NavX;
import org.usfirst.frc.team4488.lib.util.PreferenceDoesNotExistException;
import org.usfirst.frc.team4488.lib.util.PreferencesParser;
import org.usfirst.frc.team4488.lib.util.app.math.Translation2d;
import org.usfirst.frc.team4488.robot.loops.Loop;
import org.usfirst.frc.team4488.robot.sensors.Potentiometer;

public class SwerveModule implements Loop {

  private CANSparkMax angleSpark;
  private CANSparkMax speedSpark;
  private CANPIDController speedController;
  private SimPID angleController;

  private Potentiometer anglePot;

  private Translation2d targetVector = new Translation2d(0, 0);
  private boolean reversed = false;

  private final double potOffset;
  private final double ticksPerRotationAngle = 4096;
  private final double wheelDiameter = 4;
  private final double throttleGearRatio = 7;

  private enum ControlMode {
    Voltage,
    Velocity;
  }

  private ControlMode controlMode = ControlMode.Voltage;

  public static class SwerveParameters {
    public int throttleControllerID;
    public int angleControllerID;
    public int anglePotID;
    public double potOffset;

    public SwerveParameters(
        int throttleControllerID, int angleControllerID, int anglePotID, double potOffset) {
      this.throttleControllerID = throttleControllerID;
      this.angleControllerID = angleControllerID;
      this.anglePotID = anglePotID;
      this.potOffset = potOffset;
    }
  }

  @Override
  public void onStart(double timestamp) {}

  @Override
  public void onLoop(double timestamp) {
    switch (controlMode) {
      case Voltage:
        double targetVoltage = targetVector.norm();
        targetVoltage *= reversed ? -1 : 1;
        speedController.setReference(targetVoltage, ControlType.kDutyCycle);
        break;
      case Velocity:
        double targetVelocity = targetVector.norm();
        targetVelocity *= reversed ? -1 : 1;
        targetVelocity = (targetVelocity * 60) / (Math.PI * wheelDiameter); // ips to rpm
        speedController.setReference(targetVelocity, ControlType.kVelocity);
        break;
    }

    double targetAngle = targetVector.direction().getRadians();
    if (reversed) {
      targetAngle += Math.PI;
      targetAngle %= 2 * Math.PI;
    }

    angleController.setDesiredValue(targetAngle / (2 * Math.PI) * ticksPerRotationAngle);
    double power = angleController.calcPID(getAngleTicks()) * -1;
    angleSpark.set(power);
  }

  @Override
  public void onStop(double timestamp) {}

  public SwerveModule(SwerveParameters parameters) {
    speedSpark = new CANSparkMax(parameters.throttleControllerID, MotorType.kBrushless);
    angleSpark = new CANSparkMax(parameters.angleControllerID, MotorType.kBrushless);
    speedController = new CANPIDController(speedSpark);
    angleController = new SimPID();
    anglePot = new Potentiometer(parameters.anglePotID);
    speedSpark.setClosedLoopRampRate(0.5);
    angleController.setWrapAround(0, 4096);
    this.potOffset = parameters.potOffset;
  }

  public void driveVoltage(Translation2d vector) {
    controlMode = ControlMode.Voltage;
    targetVector = vector;

    reversed = shouldReverse(vector);
  }

  public void driveVelocity(Translation2d vector) {
    controlMode = ControlMode.Velocity;
    targetVector = vector;

    reversed = shouldReverse(vector);
  }

  private boolean shouldReverse(Translation2d vector) {
    double currentTicks = getAngleTicks();
    double angleTicks = (vector.direction().getRadians() / (2 * Math.PI)) * ticksPerRotationAngle;
    double error = Math.IEEEremainder(angleTicks - currentTicks, ticksPerRotationAngle);

    boolean shouldReverse = Math.abs(error) > 0.25 * ticksPerRotationAngle;
    return shouldReverse;
  }

  public double getAngleAnalog() {
    return anglePot.get();
  }

  public double getAngleTicks() {
    return (anglePot.get() - potOffset + ticksPerRotationAngle) % ticksPerRotationAngle;
  }

  public double getAngle() {
    return (getAngleTicks() * 2 * Math.PI / ticksPerRotationAngle);
  }

  public double getFieldAngle() {
    return (getAngle() - NavX.getInstance().getYaw().getRadians()) % (2 * Math.PI);
  }

  public double getSpeedNative() {
    return speedSpark.getEncoder().getVelocity(); // get speed from spark
  }

  public double getSpeed() {
    return (getSpeedNative() / 60) * Math.PI * wheelDiameter;
  }

  public double getDistanceDriven() {
    return speedSpark.getEncoder().getPosition() * Math.PI * wheelDiameter / throttleGearRatio;
  }

  public void updatePrefs() {
    PreferencesParser prefs = PreferencesParser.getInstance();

    try {
      speedController.setP(prefs.getDouble("SwerveThrottleP"));
      speedController.setI(prefs.getDouble("SwerveThrottleI"));
      speedController.setD(prefs.getDouble("SwerveThrottleD"));
      speedController.setFF(prefs.getDouble("SwerveThrottleF"));

      double p = prefs.getDouble("SwerveAngleP");
      double i = prefs.getDouble("SwerveAngleI");
      double d = prefs.getDouble("SwerveAngleD");
      angleController.setConstants(p, i, d);
    } catch (PreferenceDoesNotExistException e) {
      speedController.setP(0);
      speedController.setI(0);
      speedController.setD(0);
      speedController.setFF(0);

      angleController.setConstants(0, 0, 0);
    }
  }

  public void stop() {
    // Threshold to make sure this doesnt run forever
    if (targetVector.norm() > 0.01) {
      targetVector = targetVector.scale(0.000001);
    }
  }
}
