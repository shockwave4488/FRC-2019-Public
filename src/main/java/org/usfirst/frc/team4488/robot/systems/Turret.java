package org.usfirst.frc.team4488.robot.systems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4488.lib.util.PreferenceDoesNotExistException;
import org.usfirst.frc.team4488.lib.util.PreferencesParser;
import org.usfirst.frc.team4488.robot.Constants;
import org.usfirst.frc.team4488.robot.RobotMap;
import org.usfirst.frc.team4488.robot.loops.Loop;
import org.usfirst.frc.team4488.robot.loops.Looper;
import org.usfirst.frc.team4488.robot.operator.Logging;
import org.usfirst.frc.team4488.robot.systems.LEDController.Color;

public class Turret extends Subsystem {

  public static Turret instance = null;

  public static Turret getInstance() {
    if (instance == null) instance = new Turret();
    return instance;
  }

  private WPI_TalonSRX talon;

  private double targetAngle = 0;
  private double ticksPerRevolution = 32512;
  private double zeroOffsetTicks = 0;
  private final double MIN_DEGREES = -270;
  private final double MAX_DEGREES = 270;
  private int stabilityCount = 0;
  double stability = 2;

  public TurretSafeState turretSafe;

  public enum TurretSafeState {
    Safe(0),
    Unsafe(1);

    public int val;

    private TurretSafeState(int val) {
      this.val = val;
    }
  }

  public Turret() {
    talon = new WPI_TalonSRX(RobotMap.TurretMotor);
    talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    talon.configNominalOutputForward(0, 0);
    talon.configNominalOutputReverse(0, 0);
    talon.configPeakOutputForward(1, 0);
    talon.configPeakOutputReverse(-1, 0);
    talon.configAllowableClosedloopError(0, 0, 0);
    talon.config_IntegralZone(0, 451);
    talon.setInverted(true);
    talon.setSensorPhase(true);

    try {
      PreferencesParser prefs = PreferencesParser.getInstance();
      talon.config_kF(0, 0, 0);
      talon.config_kP(0, prefs.getDouble("TurretP"), 0);
      talon.config_kI(0, prefs.getDouble("TurretI"), 0);
      talon.config_kD(0, prefs.getDouble("TurretD"), 0);

    } catch (PreferenceDoesNotExistException e) {
      talon.config_kF(0, 0, 0);
      talon.config_kP(0, 0.14, 0);
      talon.config_kI(0, 0, 0);
      talon.config_kD(0, 0, 0);
    }
  }

  private Loop loop =
      new Loop() {
        @Override
        public void onStart(double timestamp) {
          talon.setNeutralMode(NeutralMode.Brake);
        }

        @Override
        public void onLoop(double timestamp) {
          if (Math.abs(getSpecificAngle() - getTargetAngle()) < stability) {
            stabilityCount++;
          } else {
            stabilityCount = 0;
          }

          talon.set(
              ControlMode.Position, ((targetAngle / 360.0) * ticksPerRevolution) + zeroOffsetTicks);
        }

        @Override
        public void onStop(double timestamp) {
          talon.setNeutralMode(NeutralMode.Coast);
        }
      };

  public void goToAngle(double angle) {
    targetAngle = Math.max(Math.min(angle, MAX_DEGREES), MIN_DEGREES);
  }

  public TurretSafeState safeState(double angle) {
    if ((angle > Constants.turretSafe1 && angle < Constants.turretSafe2)
        || (angle < Constants.turretSafe3 && angle > Constants.turretSafe4)
        || (angle > Constants.turretSafe5 && angle < Constants.turretSafe6)
        || (angle > Constants.turretSafe7 && angle < Constants.turretSafe8)
        || (angle < Constants.turretSafe9 && angle > Constants.turretSafe10)
        || (angle > Constants.turretSafe11 && angle < Constants.turretSafe12)
        || (angle < Constants.turretSafe13 && angle > Constants.turretSafe14)) {
      turretSafe = TurretSafeState.Safe;
    } else {
      turretSafe = TurretSafeState.Unsafe;
    }
    return turretSafe;
  }

  public double getTargetAngle() {
    return targetAngle;
  }

  // -360 to 360
  public double getSpecificAngle() {
    return ((talon.getSelectedSensorPosition() - zeroOffsetTicks) / ticksPerRevolution) * 360;
  }

  public boolean isCurrentThresholdPassed() {
    return talon.getOutputCurrent() >= Constants.turretCurrentThreshold;
  }

  // 0 to 360
  /*public double getAngle() {
    return (360 - getSpecificAngle()) % 360;
  }*/

  @Override
  public void writeToLog() {}

  @Override
  public void updateSmartDashboard() {
    SmartDashboard.putNumber("Turret Angle", getSpecificAngle());
    SmartDashboard.putNumber("Turret Des", getTargetAngle());
    SmartDashboard.putNumber("Turret Goal", targetAngle);
  }

  @Override
  public void stop() {}

  @Override
  public void zeroSensors() {
    talon.getSensorCollection().setQuadraturePosition(0, 0);
  }

  public boolean isStable(int stabilityRange) {
    return stabilityCount > stabilityRange;
  }

  public void setStabilityRange(double range) {
    stability = range;
  }

  @Override
  public void registerEnabledLoops(Looper enabledLooper) {
    enabledLooper.register(loop);
  }

  @Override
  public void updatePrefs() {}

  @Override
  public void reset() {}

  public void zeroAtCurrent() {
    Logging.getInstance().writeToLogFormatted(this, "Zeroed Turret");
    zeroOffsetTicks = talon.getSelectedSensorPosition();
    targetAngle = 0;
    LEDController.getInstance().blinkColor(Color.Purple);
  }
}
