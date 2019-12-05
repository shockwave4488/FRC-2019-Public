package org.usfirst.frc.team4488.robot.autonomous.actions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4488.robot.Constants;
import org.usfirst.frc.team4488.robot.operator.Controllers;
import org.usfirst.frc.team4488.robot.operator.Logging;
import org.usfirst.frc.team4488.robot.systems.Arm;
import org.usfirst.frc.team4488.robot.systems.Camera;
import org.usfirst.frc.team4488.robot.systems.Drive;
import org.usfirst.frc.team4488.robot.systems.Drive.LineSensors;
import org.usfirst.frc.team4488.robot.systems.Manipulator;
import org.usfirst.frc.team4488.robot.systems.SmartDrive;
import org.usfirst.frc.team4488.robot.systems.SubsystemManager;
import org.usfirst.frc.team4488.robot.systems.Turret;

public class LinePanelScore implements Action {

  public static LinePanelScore inst;

  private enum Stage {
    FindingLine,
    WaitForDrive,
    ArmPositioning,
    Scoring,
    Retracting,
    Done
  }

  private Stage stage = Stage.FindingLine;

  private Stage oldStage = Stage.Done;

  private Arm arm = Arm.getInstance();
  private Manipulator manip = Manipulator.getInstance();
  private Camera cam = Camera.getInstance();
  private SubsystemManager manager = SubsystemManager.getInstance();
  private Turret turret = Turret.getInstance();
  private Controllers xbox = Controllers.getInstance();
  private Drive drive = Drive.getInstance();
  private SmartDrive smartDrive = new SmartDrive(drive);

  private int highCurrentCycles = 0;
  private int stationaryCycles = 0;
  private double lastLength = 0;
  private double driveTrigger;
  private boolean currentTriggered = false;
  private double releaseTime = 0;
  private boolean released;
  private double turretTrigger;
  private final int triggerCurrentCycles = 5;
  private final double driveTriggerOffset = 4;
  private final double driveThrottle = 0.15;
  private boolean isAuto = false;

  public LinePanelScore(boolean isAuto) {
    this.isAuto = isAuto;
  }

  public static void teleRestart() {
    inst = new LinePanelScore(false);
    inst.start();
  }

  public static void teleUpdate() {
    inst.update();
  }

  public static void teleDone() {
    inst.done();
  }

  public static boolean teleIsFinished() {
    return inst.isFinished();
  }

  @Override
  public void update() {
    if (stage != oldStage) {
      Logging.getInstance().writeToLogFormatted("line panel scoring", stage.name());
    }
    oldStage = stage;
    switch (stage) {
      case FindingLine:
        if ((drive.seesLine(LineSensors.LeftForward) || drive.seesLine(LineSensors.RightForward))
            && (drive.getAngle() > -5 && drive.getAngle() < 5)) {
          driveTrigger = drive.getLinearDistance() + driveTriggerOffset;
          if (drive.seesLine(LineSensors.RightForward)) {
            turretTrigger = 90;
          } else if (drive.seesLine(LineSensors.LeftForward)) {
            turretTrigger = -90;
          }
          smartDrive.driveToDistance(driveTrigger);
          stage = Stage.WaitForDrive;
        } else {
          drive.driveStraightField(driveThrottle);
        }
        break;

      case WaitForDrive:
        manager.turnTurretToField(turretTrigger);
        smartDrive.driveToDistance(driveTrigger);
        manager.safeAngleArm(15);
        manager.safeExtendArm(0);
        if (smartDrive.isDriveDistanceDone() && turret.isStable(5)) {
          stage = stage.Scoring;
        }
        break;
      case ArmPositioning:
        smartDrive.driveToDistance(driveTrigger);
        manager.safeExtendArm(6);
        manager.safeAngleArm(Constants.armAngleThreshLowMid + 1);
        if (Math.abs(arm.getAngle() + 1) < 4 && Math.abs(arm.getLength() - 6) < 1) {
          stage = stage.Scoring;
        }
        break;

      case Scoring:
        manager.safeAngleArm(0);
        smartDrive.driveToDistance(driveTrigger);
        arm.directSetPower(0.6);
        manip.releasePanel();
        if (arm.isCurrentPlacementThresholdPassed()) {
          highCurrentCycles++;
        } else {

          highCurrentCycles = 0;
        }
        if (highCurrentCycles > triggerCurrentCycles) {
          SmartDashboard.putNumber("Dist Change", Math.abs(arm.getLength() - lastLength));
          if (currentTriggered
              && Math.abs(arm.getLength() - lastLength) < Constants.armStationaryRange) {
            stationaryCycles++;
            if (stationaryCycles > 3) {
              /*
              manip.releasePanel();
              if (!released) {
                releaseTime = Timer.getFPGATimestamp() * 1000;
                released = true;
              }
              */
              stage = Stage.Retracting;
              arm.directSetPower(0);
            }
          } else {
            stationaryCycles = 0;
          }
          currentTriggered = true;
          lastLength = arm.getLength();
        } else {
          stationaryCycles = 0;
          currentTriggered = false;
        }
        /*
        if (((Timer.getFPGATimestamp() * 1000) - releaseTime) > 300 && released) {
          stage = Stage.Retracting;
          arm.directSetPower(0);
        }
        */
        if (arm.maxLimitTripped()) {
          stage = Stage.Retracting;
          arm.directSetPower(0);
        }

        break;

      case Retracting:
        smartDrive.driveToDistance(driveTrigger);
        cam.ledOff();
        if (isAuto) {
          arm.directSetPower(-0.3);
          stage = Stage.Done;
        } else {
          manager.safeAngleArm(Constants.armAngleThreshLowMid + 1);
          manager.safeExtendArm(0);
          if (Math.abs(arm.getLength() - 0) < 2) {
            stage = Stage.Done;
          }
        }
        manager.resetFieldTurretAngle();
        break;

      case Done:
        break;
    }
  }

  @Override
  public void done() {}

  @Override
  public void start() {}

  @Override
  public boolean isFinished() {
    return stage == Stage.Done;
  }
}
