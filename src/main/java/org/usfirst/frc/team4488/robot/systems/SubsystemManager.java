package org.usfirst.frc.team4488.robot.systems;

import java.util.ArrayList;
import org.usfirst.frc.team4488.robot.RobotSystems;
import org.usfirst.frc.team4488.robot.loops.Looper;
import org.usfirst.frc.team4488.robot.operator.Controllers;
import org.usfirst.frc.team4488.robot.operator.Controllers.SecondaryState;
import org.usfirst.frc.team4488.robot.operator.Logging;
import org.usfirst.frc.team4488.robot.systems.drive.DriveBase;

// import org.usfirst.frc.team4488.robot.systems.LEDController.Color;

/** Used to reset, start, stop, and update all subsystems at once */
public class SubsystemManager {

  private final Controllers xbox = Controllers.getInstance();
  private final ArrayList<Subsystem> mAllSubsystems;
  private final DriveBase drive;

  public enum RobotStates {
    Manual
  }

  private RobotStates state;

  private static SubsystemManager sInstance;

  public static SubsystemManager createInstance(RobotSystems robotSystems) {
    ArrayList<Subsystem> systems = robotSystems.otherSystems;
    systems.add((Subsystem) robotSystems.drive);
    sInstance = new SubsystemManager(robotSystems.drive, systems);
    return sInstance;
  }

  public static SubsystemManager getInstance() {
    return sInstance;
  }

  public SubsystemManager(DriveBase drive, ArrayList<Subsystem> allSubsystems) {
    mAllSubsystems = allSubsystems;
    state = RobotStates.Manual;
    this.drive = drive;
  }

  public void updateSmartDashboard() {
    mAllSubsystems.forEach((s) -> s.updateSmartDashboard());
  }

  public void writeToLog() {
    mAllSubsystems.forEach((s) -> s.writeToLog());
  }

  public void stop() {
    mAllSubsystems.forEach((s) -> s.stop());
  }

  public void zeroSensors() {
    mAllSubsystems.forEach((s) -> s.zeroSensors());
  }

  public void registerEnabledLoops(Looper enabledLooper) {
    mAllSubsystems.forEach((s) -> s.registerEnabledLoops(enabledLooper));
  }

  public void updatePrefs() {
    mAllSubsystems.forEach((s) -> s.updatePrefs());
  }

  public void reset() {
    mAllSubsystems.forEach((s) -> s.reset());
  }

  public void controllerUpdates() {

    if (xbox.getSelect(xbox.m_secondary)) {
      state = RobotStates.Manual;
    }

    switch (state) {
      case Manual:
        manualUpdate();
        break;
      default:
        break;
    }
  }

  private void manualUpdate() {

    // Manual and assisted controls

    if (xbox.getStart(xbox.m_secondary)) {
      xbox.secondaryState = SecondaryState.auto;
      Logging.getInstance().writeToLogFormatted(SecondaryState.class, xbox.secondaryState.name());
    } else if (xbox.getSelect(xbox.m_secondary)) {
      xbox.secondaryState = SecondaryState.manual;
      Logging.getInstance().writeToLogFormatted(this, xbox.secondaryState.name());
    }

    /*
    if (xbox.getA(xbox.m_primary)) {
      SpiderBotClimb.getInstance().spiderDown();
    }
    if (xbox.getB(xbox.m_primary)) {
      SpiderBotClimb.getInstance().spiderUp();
    }
    */

    double forward = xbox.deadzone(xbox.getLeftStickY(xbox.m_primary), 0.2);
    double strafe = xbox.deadzone(xbox.getLeftStickX(xbox.m_primary), 0.2);
    double turn = xbox.deadzone(xbox.getRightStickX(xbox.m_primary), 0.2);
    drive.controllerUpdate(forward, strafe, turn);

    if (xbox.secondaryState == SecondaryState.auto) {
      // Auto controls
    } else {
      // Manual controls
    }
  }
}
