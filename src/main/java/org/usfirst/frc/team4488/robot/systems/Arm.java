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

// migrated to new branch

public class Arm extends Subsystem {

  private static Arm arm = Arm.getInstance();

  public static Arm getInstance() {
    if (arm == null) {
      arm = new Arm();
    }
    return arm;
  }

  private WPI_TalonSRX armExtender;
  private WPI_TalonSRX armAngler;

  private double desiredExtension;
  private double setpointRamp = 1; // +/- x degrees every cycle, max
  private double desiredAngle;
  private double directPower = 0;
  private final double ticksPerInch = 861.63; // wrong
  private final double zeroAngleOffset; // 750.25;
  private final double armAngleMax;
  private final double armExtenderMin;
  private final double armExtenderMax;
  private final double ticksPerDegree;
  private final double cargoIntakeAngle;

  private double extOffset = 0;

  private boolean armDead;

  public ArmExtendState armExtendState;
  public ArmAngleState armAngleState;

  public enum ArmExtendState {
    Short(0),
    Mid1(1),
    Mid2(2),
    Long(3);

    public int val;

    private ArmExtendState(int val) {
      this.val = val;
    }
  }

  public enum ArmAngleState {
    Low(0),
    Mid(1),
    MidMid(2),
    High(3);

    public int val;

    private ArmAngleState(int val) {
      this.val = val;
    }
  }

  public Arm() {

    armExtender = new WPI_TalonSRX(RobotMap.armExtender);
    armAngler = new WPI_TalonSRX(RobotMap.armAngler);

    armExtender.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    armExtender.configNominalOutputForward(0, 0);
    armExtender.configNominalOutputReverse(0, 0);
    armExtender.configPeakOutputForward(1, 0);
    armExtender.configPeakOutputReverse(-1, 0);
    armExtender.configAllowableClosedloopError(0, 0, 0);
    armExtender.setNeutralMode(NeutralMode.Brake);
    armExtender.setInverted(true);
    armExtender.setSensorPhase(false);
    armExtender.config_IntegralZone(0, 1600);

    armAngler.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
    armAngler.configNominalOutputForward(0, 0);
    armAngler.configNominalOutputReverse(0, 0);
    armAngler.configPeakOutputForward(1, 0);
    armAngler.configPeakOutputReverse(-1, 0);
    armAngler.configAllowableClosedloopError(0, 0, 0);
    armAngler.setNeutralMode(NeutralMode.Brake);
    armAngler.setInverted(false);
    armAngler.setSensorPhase(false);
    armAngler.config_IntegralZone(0, 20);

    PreferencesParser prefs = PreferencesParser.getInstance();
    try {
      armExtender.config_kP(0, prefs.getDouble("ArmExtenderP"));
      armExtender.config_kI(0, prefs.getDouble("ArmExtenderI"));
      armExtender.config_kD(0, prefs.getDouble("ArmExtenderD"));
      armExtender.config_kF(0, 0);

      armAngler.config_kP(0, prefs.getDouble("ArmAnglerP"));
      armAngler.config_kI(0, prefs.getDouble("ArmAnglerI"));
      armAngler.config_kD(0, prefs.getDouble("ArmAnglerD"));
      armAngler.config_kF(0, 0);

    } catch (PreferenceDoesNotExistException e) {
      armExtender.config_kP(0, Constants.armExtenderP);
      armExtender.config_kI(0, Constants.armExtenderI);
      armExtender.config_kD(0, Constants.armExtenderD);
      armExtender.config_kF(0, 0);

      armAngler.config_kP(0, Constants.armAnglerP);
      armAngler.config_kI(0, Constants.armAnglerI);
      armAngler.config_kD(0, Constants.armAnglerD);
      armExtender.config_kF(0, 0);
    }

    zeroAngleOffset = prefs.getDouble("zeroAngleOffset");
    armAngleMax = prefs.getDouble("armAngleMax");
    armExtenderMin = prefs.getDouble("armExtenderMin");
    armExtenderMax = prefs.getDouble("armExtenderMax");
    ticksPerDegree = prefs.getDouble("armAngleTicksPerDegree");
    cargoIntakeAngle = prefs.getDouble("cargoIntakeAngle");

    desiredExtension = 3;
    desiredAngle = 9;
  }

  private Loop loop =
      new Loop() {
        @Override
        public void onStop(double timestamp) {}

        @Override
        public void onStart(double timestamp) {}

        @Override
        public void onLoop(double timestamp) {

          if (desiredAngle > armAngleMax) {
            desiredAngle = armAngleMax;
          }

          if(armExtender.getSensorCollection().isRevLimitSwitchClosed()) {
            extOffset = armExtender.getSelectedSensorPosition();
          }

          if (getAngle() < -30) {
            armAngler.set(ControlMode.PercentOutput, 0);
          } else {

            // dead code
            /*double d = Math.min(setpointRamp, Math.abs(desiredAngle - getAngle()));
            double target = getAngle();

            //dead code
            if (getAngle() < desiredAngle) {
              target += d;
            } else {
              target -= d;
            }*/

            // System.out.println(target);
            if (directPower != 0) {
              armExtender.set(ControlMode.PercentOutput, directPower);
            } else {
              armExtender.set(ControlMode.Position, (desiredExtension * ticksPerInch) + extOffset);
            }
            armAngler.set(ControlMode.Position, (desiredAngle * ticksPerDegree) + zeroAngleOffset);
          }
        }
      };

  public void setLength(double extension) {
    if (extension > armExtenderMax) {
      extension = armExtenderMax;
    } else if (extension < armExtenderMin) {
      extension = armExtenderMin;
    }

    desiredExtension = extension;
    directPower = 0;
  }

  public double getCargoIntakeAngle() {
    return cargoIntakeAngle;
  }

  public double getArmExtendMax() {
    return armExtenderMax;
  }

  public void setAngle(double angle) {
    if (angle > armAngleMax) {
      angle = armAngleMax;
    } else if (angle < Constants.armAngleMin) {
      angle = Constants.armAngleMin;
    }

    desiredAngle = angle;
  }

  public ArmExtendState extSafeState(double ext) {
    if (ext < Constants.armExtendLowMid1) {
      armExtendState = ArmExtendState.Short;
    } else if (ext >= Constants.armExtendLowMid1 && ext < Constants.armExtendMid1Mid2) {
      armExtendState = ArmExtendState.Mid1;
    } else if (ext >= Constants.armExtendMid1Mid2 && ext <= Constants.armExtendMid2High) {
      armExtendState = ArmExtendState.Mid2;
    } else if (ext > Constants.armExtendMid2High) {
      armExtendState = ArmExtendState.Long;
    }
    return armExtendState;
  }

  public ArmAngleState angSafeState(double ang) {
    if (ang < Constants.armAngleThreshLowMid) {
      armAngleState = ArmAngleState.Low;
    } else if (ang >= Constants.armAngleThreshLowMid && ang < Constants.armAngleThreshMidMid) {
      armAngleState = ArmAngleState.Mid;
    } else if (ang >= Constants.armAngleThreshMidMid && ang < Constants.armAngleThreshMidHigh) {
      armAngleState = ArmAngleState.MidMid;
    } else if (ang >= Constants.armAngleThreshMidHigh) {
      armAngleState = ArmAngleState.High;
    }
    return armAngleState;
  }

  public double getDesiredAngle() {
    return desiredAngle;
  }

  /** @return the desiredExtension */
  public double getLength() {
    return (armExtender.getSelectedSensorPosition() - extOffset) / ticksPerInch;
  }

  /** @return the desiredAngle */
  public double getAngle() {
    return (armAngler.getSelectedSensorPosition() - zeroAngleOffset) / ticksPerDegree;
  }

  /** @return the desiredAngle */
  public double getDesiredLength() {
    return desiredExtension;
  }

  public boolean isCurrentThresholdPassed() {
    return armExtender.getOutputCurrent() >= Constants.armExtendHighCurrent;
  }

  public boolean isCurrentPlacementThresholdPassed() {
    return armExtender.getOutputCurrent() >= Constants.armExtendPlacementCurrent;
  }

  public double armExtenderOutputCurrent() {
    return armExtender.getOutputCurrent();
  }

  @Override
  public void writeToLog() {}

  @Override
  public void updateSmartDashboard() {
    // System.out.println(armAngler.getSelectedSensorPosition());
    SmartDashboard.putNumber("arm current", armExtender.getOutputCurrent());
    SmartDashboard.putNumber("desired length", desiredExtension);
    SmartDashboard.putNumber("desired angle", desiredAngle);
    SmartDashboard.putNumber(
        "current angle",
        (armAngler.getSelectedSensorPosition() - zeroAngleOffset) / ticksPerDegree);
    SmartDashboard.putNumber(
        "current length", getLength());
    SmartDashboard.putNumber("arm angle ticks", armAngler.getSelectedSensorPosition());
  }

  @Override
  public void stop() {
    armExtender.stopMotor();
    armAngler.stopMotor();
  }

  @Override
  public void zeroSensors() {
    armExtender.getSensorCollection().setQuadraturePosition(0, 0);
  }

  @Override
  public void registerEnabledLoops(Looper enabledLooper) {
    enabledLooper.register(loop);
  }

  @Override
  public void updatePrefs() {}

  @Override
  public void reset() {}

  /** Do a lot of trig to move the arm to score at a given height */
  public double[] calcFromCamera(double camDist, double height) {
    height -= 19; // Arm pivot is 19 inches off ground
    double horizontalDistance = camDist * 0.99254615; // = sin(83), angle of camera mount
    double armExtension =
        Math.sqrt((height * height) + (horizontalDistance * horizontalDistance))
            - 27; // approximate distance from pivot on arm to end of arm
    double armAngle = Math.toDegrees(Math.atan(height / horizontalDistance));

    return new double[] {armExtension, armAngle};
  }

  public void directSetPower(double power) {
    directPower = power;
  }

  public boolean maxLimitTripped() {
    return armExtender.getSensorCollection().isFwdLimitSwitchClosed();
  }
}
