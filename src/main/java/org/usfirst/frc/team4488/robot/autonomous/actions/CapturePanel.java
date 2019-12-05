package org.usfirst.frc.team4488.robot.autonomous.actions;

import JavaRoboticsLib.ControlSystems.SimPID;
import org.usfirst.frc.team4488.lib.util.PreferenceDoesNotExistException;
import org.usfirst.frc.team4488.lib.util.PreferencesParser;
import org.usfirst.frc.team4488.robot.operator.Controllers;
import org.usfirst.frc.team4488.robot.systems.Arm;
import org.usfirst.frc.team4488.robot.systems.Camera;
import org.usfirst.frc.team4488.robot.systems.Drive;
import org.usfirst.frc.team4488.robot.systems.Manipulator;
import org.usfirst.frc.team4488.robot.systems.SubsystemManager;
import org.usfirst.frc.team4488.robot.systems.Turret;

public class CapturePanel implements Action {

  private Turret turret = Turret.getInstance();
  private Arm arm = Arm.getInstance();
  private Manipulator manip = Manipulator.getInstance();
  private Drive drive = Drive.getInstance();
  private Camera cam = Camera.getInstance();
  private Controllers xbox = Controllers.getInstance();
  private SubsystemManager manager = SubsystemManager.getInstance();

  private SimPID turnController = new SimPID();

  private boolean isAuto;
  private double inRangeCycles = 0;

  private final double defaultP = 0.0021875;
  private final double defaultI = 0;
  private final double defaultD = 0;
  private final double armExt = 12.1;
  private final double armAngle = -5;
  private final double armDoneRange = 0.6;
  private final double throttleScale = -1.4;
  private final double cycleThreshold = 10;

  private enum Stage {
    ExtendingArm,
    Capturing,
    Done
  }

  private Stage stage = Stage.ExtendingArm;

  private static CapturePanel inst;

  public CapturePanel(boolean isAuto) {
    this.isAuto = isAuto;

    PreferencesParser prefs = PreferencesParser.getInstance();
    try {
      turnController.setConstants(
          prefs.getDouble("VisionTurnControllerP"),
          prefs.getDouble("VisionTurnControllerI"),
          prefs.getDouble("VisionTurnControllerD"));
    } catch (PreferenceDoesNotExistException e) {
      turnController.setConstants(defaultP, defaultI, defaultD);
    }
  }

  public static void teleRestart() {
    inst = new CapturePanel(false);
  }

  public static void teleStart() {
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
  public void start() {
    drive.configPercentVbus();
  }

  @Override
  public void update() {
    switch (stage) {
      case ExtendingArm:
        setSetpoints();

        if (Math.abs(arm.getLength() - armExt) < armDoneRange) {
          inRangeCycles++;
        } else {
          inRangeCycles = 0;
        }
        if(inRangeCycles > cycleThreshold) {
          stage = Stage.Capturing;
        }
        break;

      case Capturing:
        setSetpoints();
        manip.releasePanel();

        cam.ledOn();
        if (cam.targetFound()) {
          turnController.setDesiredValue(cam.cameraWidthPixels / 2);
          double powerMod = turnController.calcPID(cam.getLastReading().x);
          double throttle =
              isAuto
                  ? throttleScale * (0.35 - powerMod)
                  : xbox.getLeftStickY(xbox.m_primary); // 0.35 = max power mod
          drive.setPowers(throttle - powerMod, throttle + powerMod);
        } else {
          manager.driveUpdate();
        }

        if (arm.isCurrentThresholdPassed()) {
          manip.holdPanel();
          manager.resetFieldTurretAngle();
          drive.setPowers(0, 0);
          stage = Stage.Done;
        }
        break;

      case Done:
        break;
    }
  }

  private void setSetpoints() {
    if (-90 < turret.getSpecificAngle() && turret.getSpecificAngle() < 90)
      manager.safeRotateTurret(0);
    else if (turret.getSpecificAngle() < -90) manager.safeRotateTurret(-180);
    else manager.safeRotateTurret(180);
    manager.safeExtendArm(armExt);
    manager.safeAngleArm(armAngle);
  }

  @Override
  public void done() {}

  @Override
  public boolean isFinished() {
    return stage == Stage.Done;
  }
}
