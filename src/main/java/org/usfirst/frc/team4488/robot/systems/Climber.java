package org.usfirst.frc.team4488.robot.systems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Solenoid;
import org.usfirst.frc.team4488.lib.util.PreferenceDoesNotExistException;
import org.usfirst.frc.team4488.lib.util.PreferencesParser;
import org.usfirst.frc.team4488.robot.Constants;
import org.usfirst.frc.team4488.robot.RobotMap;
import org.usfirst.frc.team4488.robot.loops.Loop;
import org.usfirst.frc.team4488.robot.loops.Looper;

public class Climber extends Subsystem {

  public static Climber instance = null;

  public static Climber getInstance() {
    if (instance == null) instance = new Climber();
    return instance;
  }

  private enum ClimberState {
    IDLE,
    CLIMBING,
    STOPPED
  }

  private ClimberState state;

  private WPI_TalonSRX master;
  private WPI_VictorSPX follower;
  private Solenoid climbPancake;

  private static final boolean climberBreakOn = false;
  private static final boolean climberBreakOff = !climberBreakOn;
  private boolean finished = false;

  private double climbPower;

  public Climber() {
    climbPancake = new Solenoid(RobotMap.ClimberPancake);
    master = new WPI_TalonSRX(RobotMap.ClimberMaster);
    follower = new WPI_VictorSPX(RobotMap.ClimberSlave);
    master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    master.configClosedloopRamp(0.5);
    follower.follow(master);

    master.configNominalOutputForward(0, 0);
    master.configNominalOutputReverse(0, 0);
    master.configPeakOutputForward(1, 0);
    master.configPeakOutputReverse(0.1, 0);
    master.configAllowableClosedloopError(0, 0, 0);
    master.setInverted(false);
    master.setSensorPhase(false);

    try {
      PreferencesParser prefs = PreferencesParser.getInstance();
      master.config_kP(0, prefs.getDouble("ClimberP"));
      master.config_kI(0, prefs.getDouble("ClimberI"));
      master.config_kD(0, prefs.getDouble("ClimberD"));
    } catch (PreferenceDoesNotExistException e) {
      master.config_kF(0, 3.7, 0);
      master.config_kP(0, 0.1, 0);
      master.config_kI(0, 0, 0);
      master.config_kD(0, 0, 0);
    }

    zeroSensors();

    state = ClimberState.IDLE;
  }

  private Loop loop =
      new Loop() {
        @Override
        public void onStart(double timestamp) {
          climbPancake.set(climberBreakOn);
          master.setNeutralMode(NeutralMode.Brake);
        }

        @Override
        public void onLoop(double timestamp) {
          switch (state) {
            case STOPPED:
              master.set(ControlMode.PercentOutput, 0);
              climbPancake.set(climberBreakOff);
              break;
            case IDLE:
              break;
            case CLIMBING:
              climbPancake.set(climberBreakOff);
              master.set(ControlMode.PercentOutput, Constants.climberPower);
              if (master.getSelectedSensorPosition() > Constants.climberDoneTicks) {
                state = ClimberState.STOPPED;
                finished = true;
              }
              break;
          }
        }

        @Override
        public void onStop(double timestamp) {
          master.setNeutralMode(NeutralMode.Brake);
        }
      };

  /** Starts climbing and continues until it finishes */
  public void deploy() {
    if (state == ClimberState.IDLE) {
      state = ClimberState.CLIMBING;
    }
  }

  public boolean isClimbing() {
    return state == ClimberState.CLIMBING;
  }

  public boolean isDone() {
    return finished;
  }

  /** Stops climbing and climbing cant be restarted after */
  public void stop() {
    if (state == ClimberState.CLIMBING) {
      state = ClimberState.STOPPED;
    }
    master.set(ControlMode.PercentOutput, 0);
  }

  public double getTicks() {
    return master.getSelectedSensorPosition();
  }

  @Override
  public void reset() {}

  @Override
  public void updateSmartDashboard() {
    // SmartDashboard.putNumber("ClimberTicks: ", master.getSelectedSensorPosition());
  }

  @Override
  public void zeroSensors() {
    master.getSensorCollection().setQuadraturePosition(0, 0);
  }

  @Override
  public void registerEnabledLoops(Looper looper) {
    looper.register(loop);
  }

  @Override
  public void writeToLog() {}

  @Override
  public void updatePrefs() {}
}
