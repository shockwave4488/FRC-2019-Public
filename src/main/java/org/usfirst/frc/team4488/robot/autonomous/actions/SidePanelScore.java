package org.usfirst.frc.team4488.robot.autonomous.actions;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4488.robot.Constants;
import org.usfirst.frc.team4488.robot.autonomous.actions.SideCargoScoreLine.Targets;
import org.usfirst.frc.team4488.robot.operator.Controllers;
import org.usfirst.frc.team4488.robot.systems.Arm;
import org.usfirst.frc.team4488.robot.systems.Camera;
import org.usfirst.frc.team4488.robot.systems.Camera.CameraFrame;
import org.usfirst.frc.team4488.robot.systems.Camera.LastTiltRead;
import org.usfirst.frc.team4488.robot.systems.Drive;
import org.usfirst.frc.team4488.robot.systems.Manipulator;
import org.usfirst.frc.team4488.robot.systems.SubsystemManager;
import org.usfirst.frc.team4488.robot.systems.Turret;

public class SidePanelScore implements Action {

  public static SidePanelScore inst;

  private enum Stage {
    Tracking,
    Scoring,
    Retracting,
    Done
  }

  private Stage stage = Stage.Tracking;

  private final double turretSpeed = 10;

  private Arm arm = Arm.getInstance();
  private Manipulator manip = Manipulator.getInstance();
  private Camera cam = Camera.getInstance();
  private SubsystemManager manager = SubsystemManager.getInstance();
  private Turret turret = Turret.getInstance();
  private Controllers xbox = Controllers.getInstance();

  private int highCurrentCycles = 0;
  private int stationaryCycles = 0;
  private double lastLength = 0;
  private boolean currentTriggered = false;
  private double releaseTime = 0;
  private boolean released;
  private final int triggerCurrentCycles = 8;
  private final int onTargetThreshold = 8;
  private final double stickDeadzone = 0.3;
  private final double onTargetDoneRangeShip = 1.5;
  private final double onTargetDoneRangeRocket = 2.5;
  private final int maximumDistance = 40;
  private int onTargetCycles = 0;
  private boolean isAuto;

  private Targets target;

  public SidePanelScore(Targets target, boolean isAuto) {
    this.target = target;
    this.isAuto = isAuto;
  }

  public static void teleRestart(Targets target) {
    inst = new SidePanelScore(target, false);
    inst.start();
  }

  public static void setTarget(Targets target) {
    inst.target = target;
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
    switch (stage) {
      case Tracking:
        manager.driveUpdate();
        manager.safeAngleArm(Constants.armAngleThreshLowMid + 0.1);
        manager.safeExtendArm(5.6);
        cam.ledOn();
        CameraFrame read = cam.getLastReading();
        double turretStick = xbox.getLeftStickX(xbox.m_secondary);
        if (cam.targetFound() && read.tilt == LastTiltRead.Both) {
          double errX = read.x - (cam.cameraWidthPixels / 2);
          double errDegrees = ((errX / (cam.cameraWidthPixels / 2)) * (cam.cameraFOV / 2)) % 360;
          errDegrees *= 0.8;
          manager.safeRotateTurret(turret.getSpecificAngle() + errDegrees);
          double doneRange =
              (target == Targets.RightRocket || target == Targets.LeftRocket)
                  ? onTargetDoneRangeRocket
                  : onTargetDoneRangeShip;

          if (Math.abs(errDegrees) < doneRange) {
            // manager.safeRotateTurret(turret.getSpecificAngle());
            onTargetCycles++;
          } else {
            onTargetCycles = 0;
          }
          if (onTargetCycles > onTargetThreshold) {
            stage = Stage.Scoring;
          }
        } else {
          manager.turretUpdate();
        }
        if (xbox.getY(xbox.m_secondary)) {
          stage = Stage.Scoring;
        }
        break;

      case Scoring:
        manager.driveUpdate();
        arm.directSetPower(0.6);
        if (arm.isCurrentPlacementThresholdPassed()) {
          highCurrentCycles++;
        } else {

          highCurrentCycles = 0;
        }
        if (highCurrentCycles > triggerCurrentCycles) {
          SmartDashboard.putNumber("Dist Change", Math.abs(arm.getLength() - lastLength));
          if (currentTriggered
              && (isAuto
                  || Math.abs(arm.getLength() - lastLength) < Constants.armStationaryRange)) {
            stationaryCycles++;
            if (stationaryCycles > 5) {
              manip.releasePanel();
              if (!released) {
                releaseTime = Timer.getFPGATimestamp() * 1000;
                released = true;
              }
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
        if (((Timer.getFPGATimestamp() * 1000) - releaseTime) > 300 && released) {
          stage = Stage.Retracting;
          arm.directSetPower(0);
        }
        if (arm.maxLimitTripped() && !released) {
          stage = Stage.Tracking;
          arm.directSetPower(0);
        }

        break;

      case Retracting:
        cam.ledOff();
        manager.safeExtendArm(4.1);
        if (Math.abs(arm.getLength() - 4.1) < 2) {
          stage = Stage.Done;
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
  public void start() {
    Drive.getInstance().configPercentVbus();
  }

  @Override
  public boolean isFinished() {
    return stage == Stage.Done;
  }
}
