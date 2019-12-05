package org.usfirst.frc.team4488.robot.systems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4488.robot.RobotMap;
import org.usfirst.frc.team4488.robot.loops.Loop;
import org.usfirst.frc.team4488.robot.loops.Looper;
import org.usfirst.frc.team4488.robot.operator.Logging;

public class SpiderBotClimb extends Subsystem {

  public static SpiderBotClimb sInstance = null;

  public static SpiderBotClimb getInstance() {
    if (sInstance == null) {
      sInstance = new SpiderBotClimb();
    }

    return sInstance;
  }

  public enum SpiderState {
    FLOORIDLE,
    PADUP,
    CLIMBING,
    HANGING,
    UNCLIMBING,
    UNVACUUM
  }

  private SpiderState state;

  private static int pSensor1Raw;
  private static double pSensor1Volts;
  private static double dumpAngle;
  private static double dumpStartAngle = 90;
  private static double dumpDropAngle = 180; // releases vacuum, value is untested
  private static int pumpMotorOn = 1;
  private static int pumpMotorOff = 0;
  private static final double VOLTAGE_THRESHOLD = 2.5;
  private static final double VOLTAGE_LIMIT = 2.0;
  private static final double WAIT_TIME = 2.0;
  private static final double SENSOR_VOLT_THRESHOLD = 0.35;

  private Talon pumpmotor;
  private AnalogInput pressureSensor1;
  private Solenoid piston1;
  private Servo dumpValve;

  private Timer unVacuumWait;
  private Timer climbingWait;

  private boolean pistonExpand = true;
  private boolean pistonCompact = !pistonExpand;

  public SpiderBotClimb() {
    pumpmotor = new Talon(RobotMap.ClimberVacuumMotor);
    pressureSensor1 = new AnalogInput(RobotMap.ClimberPressureSensor);
    piston1 = new Solenoid(RobotMap.ClimberSolenoid);
    dumpValve = new Servo(RobotMap.ClimberServo);
    unVacuumWait = new Timer();
    climbingWait = new Timer();
    state = SpiderState.FLOORIDLE;
  }

  private Loop spiderLoop =
      new Loop() {
        @Override
        public void onStart(double timestamp) {}

        @Override
        public void onLoop(double timestamp) {
          pSensor1Raw = pressureSensor1.getValue();
          pSensor1Volts = pressureSensor1.getVoltage();
          SmartDashboard.putNumber("Voltage 1", pSensor1Volts);
          SmartDashboard.putNumber("Raw 1", pSensor1Raw);
          dumpAngle = dumpValve.getAngle();
          SmartDashboard.putNumber("Dump Angle: ", dumpAngle);
          statePrinter();
          dumpMotorAngle();

          switch (state) {
            case FLOORIDLE:
              pumpmotor.set(pumpMotorOff);
              piston1.set(pistonCompact);
              dumpValve.setAngle(dumpStartAngle);
              break;
            case PADUP:
              dumpValve.setAngle(dumpStartAngle);
              pumpmotor.set(pumpMotorOn);
              piston1.set(pistonExpand);
              if (pSensor1Volts > VOLTAGE_THRESHOLD) {
                climbingWait.start();
                state = SpiderState.CLIMBING;
                Logging.getInstance()
                    .writeToLogFormatted(this, "Climb", "SuctionClimb", state.name());
              }
              break;
            case CLIMBING:
              // 2.7 is coasting voltage
              pumpmotor.set(pumpMotorOn);
              piston1.set(pistonCompact);
              double climbingWaitTime = climbingWait.get();
              if (pSensor1Volts > VOLTAGE_THRESHOLD) {
                pumpmotor.set(pumpMotorOff);
              } else if (pSensor1Volts < VOLTAGE_LIMIT) {
                pumpmotor.set(pumpMotorOn);
              }
              SmartDashboard.putNumber("Climbing Wait Time: ", climbingWaitTime);
              if (climbingWaitTime > WAIT_TIME) {
                climbingWait.stop();
                climbingWait.reset();
                state = SpiderState.HANGING;
                Logging.getInstance()
                    .writeToLogFormatted(this, "Climb", "SuctionClimb", state.name());
              }

              break;
            case HANGING:
              pumpmotor.set(pumpMotorOn);
              piston1.set(pistonCompact);
              if (pSensor1Volts > VOLTAGE_THRESHOLD) {
                pumpmotor.set(pumpMotorOff);
              } else if (pSensor1Volts < VOLTAGE_LIMIT) {
                pumpmotor.set(pumpMotorOn);
              }
              break;
            case UNCLIMBING:
              pumpmotor.set(pumpMotorOn);
              dumpValve.setAngle(dumpStartAngle);
              piston1.set(pistonExpand);
              double unVacuumTime = unVacuumWait.get();
              if (unVacuumTime > WAIT_TIME) {
                unVacuumWait.stop();
                unVacuumWait.reset();
                state = SpiderState.UNVACUUM;
                Logging.getInstance()
                    .writeToLogFormatted(this, "Climb", "SuctionClimb", state.name());
              }
              break;
            case UNVACUUM:
              pumpmotor.set(pumpMotorOff);
              dumpValve.setAngle(dumpDropAngle);
              piston1.set(pistonExpand);
              if (pSensor1Volts < SENSOR_VOLT_THRESHOLD) {
                state = SpiderState.FLOORIDLE;
                Logging.getInstance()
                    .writeToLogFormatted(this, "Climb", "SuctionClimb", state.name());
              }
              break;
          }
        }

        @Override
        public void onStop(double timestamp) {}
      };

  public void statePrinter() {
    SmartDashboard.putString("State: ", state.name());
  }

  public void spiderUp() {
    if (state == SpiderState.FLOORIDLE) {
      state = SpiderState.PADUP;
      Logging.getInstance().writeToLogFormatted(this, "Climb", "SuctionClimb", state.name());
    }
  }

  public void spiderDown() {
    if (state == SpiderState.HANGING) {
      unVacuumWait.start(); // (maybe true -->) timer is not working, it seems to be the issue
      state = SpiderState.UNCLIMBING;
      Logging.getInstance().writeToLogFormatted(this, "Climb", "SuctionClimb", state.name());
    }
  }

  public void dumpMotorAngle() {
    SmartDashboard.putNumber("DumpMotor Angle: ", dumpAngle);
  }

  public void pressureSensorDashboard() {
    SmartDashboard.putNumber("Voltage 1", pSensor1Volts);
    SmartDashboard.putNumber("Raw 1", pSensor1Raw);
  }

  @Override
  public void stop() {}

  @Override
  public void updateSmartDashboard() {}

  @Override
  public void zeroSensors() {}

  @Override
  public void registerEnabledLoops(Looper enabledLooper) {
    enabledLooper.register(spiderLoop);
  }

  @Override
  public void updatePrefs() {}

  @Override
  public void writeToLog() {}

  @Override
  public void reset() {}
}
