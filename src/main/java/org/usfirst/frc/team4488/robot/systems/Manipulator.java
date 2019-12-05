package org.usfirst.frc.team4488.robot.systems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import org.usfirst.frc.team4488.robot.RobotMap;
import org.usfirst.frc.team4488.robot.loops.Loop;
import org.usfirst.frc.team4488.robot.loops.Looper;

public class Manipulator extends Subsystem {
  // Defining variables with what they are (motor/solenoid)
  private WPI_VictorSPX intakeMotor;
  private Solenoid wrist;
  private Solenoid panelGrabber;
  private DigitalInput cargoPresent;
  private DigitalInput panelPresent;
  private DigitalInput againstWall;
  private static final boolean SCISSOROPEN = false;
  private static final boolean SCISSORCLOSED = !SCISSOROPEN;
  private int cargoCycles = 0;
  // Deleting any old instance, then making the new instance Manipulator
  public static Manipulator instance = null;

  private enum IntakeState {
    In,
    Out,
    SlowOut,
    Off
  }

  private IntakeState intakeState = IntakeState.Off;

  private Loop loop =
      new Loop() {
        @Override
        public void onStart(double timestamp) {
          intakeMotor.setNeutralMode(NeutralMode.Brake);
        }

        @Override
        public void onLoop(double timestamp) {
          if (!cargoPresent.get()) {
            cargoCycles++;
          } else {
            cargoCycles = 0;
          }
          switch (intakeState) {
            case In:
              if (!hasCargo()) {
                intakeMotor.set(0.5);
              } else {
                intakeMotor.set(0.16);
              }
              break;
            case Out:
              intakeMotor.set(-0.9);
              break;
            case SlowOut:
              intakeMotor.set(-0.7);
              break;
            case Off:
              if (hasCargo()) {
                intakeMotor.set(0.16);
              } else {
                intakeMotor.set(0);
              }
              break;
          }
        }

        @Override
        public void onStop(double timestamp) {}
      };

  public static Manipulator getInstance() {
    if (instance == null) instance = new Manipulator();
    return instance;
  }
  // Constructor: Internally keeps information that will be used later
  public Manipulator() {
    intakeMotor = new WPI_VictorSPX(RobotMap.Intake);
    panelGrabber = new Solenoid(RobotMap.PanelGrabber);
    cargoPresent = new DigitalInput(RobotMap.CargoSensor);
  }
  // Opens the scissor panel manipulator (which scores panels and grabs them)
  public void holdPanel() {
    panelGrabber.set(SCISSOROPEN);
  }

  // Closes the scissors so the panel is released
  public void releasePanel() {
    panelGrabber.set(SCISSORCLOSED);
  }

  // Senses if the scissors is holding a panel
  public boolean isHoldingPanel() {
    return ((panelGrabber.get() == SCISSOROPEN));
  }

  // Raises the wrist so panels and cargo can be scored
  public void raiseWrist() {
    wrist.set(true);
  }

  // Lowers the wrist so panels can be picked up from the floor
  public void lowerWrist() {
    wrist.set(false);
  }

  // Senses if the wrist is raised
  public boolean wristIsRaised() {
    return (wrist.get() == true);
  }

  // Makes it so intake wheels rotate inwards to pick up cargo and panels
  public void intakeIn() { // Assuming positive values rotate wheels inwards
    intakeState = IntakeState.In;
  }

  // Makes it so intake wheels rotate outwards to release cargo
  public void intakeOut() { // Assuming negative values rotate wheels outwards
    intakeState = IntakeState.Out;
  }

  public void intakeOutSlow() {
    intakeState = IntakeState.SlowOut;
  }

  // Turns the intake wheels off
  public void intakeOff() {
    intakeState = IntakeState.Off;
  }

  // Senses if the manipulator has possession of cargo
  public boolean hasCargo() {
    return cargoCycles > 25;
  }

  // Senses if the manipulator has possession of a panel
  public boolean hasPanel() {
    return (panelPresent.get() == true);
  }

  // Senses if the manipulator is pressing against a wall
  public boolean isAgainstWall() {
    return (againstWall.get() == true);
  }

  public void writeToLog() {}

  public void updateSmartDashboard() {}

  public void stop() {}

  public void zeroSensors() {}

  public void registerEnabledLoops(Looper enabledLooper) {
    enabledLooper.register(loop);
  }

  public void updatePrefs() {}

  public void reset() {}
}
