package org.usfirst.frc.team4488.robot.autonomous.actions;

import JavaRoboticsLib.Drive.DriveHelper;
import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team4488.robot.operator.Controllers;
import org.usfirst.frc.team4488.robot.systems.Arm;
import org.usfirst.frc.team4488.robot.systems.Camera;
import org.usfirst.frc.team4488.robot.systems.Camera.CameraFrame;
import org.usfirst.frc.team4488.robot.systems.Camera.LastTiltRead;
import org.usfirst.frc.team4488.robot.systems.Drive;
import org.usfirst.frc.team4488.robot.systems.LEDController;
import org.usfirst.frc.team4488.robot.systems.LEDController.Color;
import org.usfirst.frc.team4488.robot.systems.Manipulator;
import org.usfirst.frc.team4488.robot.systems.SubsystemManager;
import org.usfirst.frc.team4488.robot.systems.Turret;

public class SideCargoScore implements Action {

  private static SideCargoScore inst;

  private enum Stage {
    Setup,
    Tracking,
    Angling,
    Moving,
    Scoring,
    Resetting,
    Done
  }

  private Stage stage = Stage.Setup;

  private final double turretSpeed = 10;
  private final double shootTime = 1000; // milliseconds

  private double scoringStart = 0;
  private double height;
  private boolean slowShoot;
  private double turretStart;
  private double[] armSetpoints;
  private int lineUpCycles = 0;

  private Controllers xbox = Controllers.getInstance();
  private SubsystemManager manager = SubsystemManager.getInstance();
  private Drive drive = Drive.getInstance();
  private Camera cam = Camera.getInstance();
  private Turret turret = Turret.getInstance();
  private Arm arm = Arm.getInstance();
  private Manipulator manip = Manipulator.getInstance();
  private DriveHelper driveHelper = new DriveHelper(drive, 0.1, 0.15);

  public SideCargoScore(double height, boolean slowShoot) {
    this.height = height;
    this.slowShoot = slowShoot;
  }

  public static void teleRestart() {
    inst = new SideCargoScore(80, false);
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

  public static void setHeight(double height) {
    inst.height = height;
  }

  public static void setSlowShoot(boolean slowShoot) {
    inst.slowShoot = slowShoot;
  }

  @Override
  public void start() {}

  @Override
  public void update() {
    switch (stage) {
      case Setup:
        manager.safeAngleArm(10);
        manager.safeExtendArm(6);

        if (Math.abs(arm.getAngle() - 10) < 5 && Math.abs(arm.getLength() - 6) < 1) {
          stage = Stage.Tracking;
        }

        break;

      case Tracking:
        double[] powers =
            drive.calcPowers(
                xbox.getLeftStickY(xbox.m_primary), xbox.getRightStickX(xbox.m_primary));
        drive.configPercentVbus();
        driveHelper.Drive(powers[0], powers[1]);

        if (xbox.getLeftBumper(xbox.m_secondary)) {
          manager.safeExtendArm(arm.getDesiredLength() - 0.4);
        }

        if (xbox.getRightBumper(xbox.m_secondary)) {
          manager.safeExtendArm(arm.getDesiredLength() + 0.4);
        }

        if (Math.abs(xbox.getRightStickY(xbox.m_secondary)) > 0.25) {
          manager.safeAngleArm(arm.getDesiredAngle() + (2 * xbox.getRightStickY(xbox.m_secondary)));
        }

        double turretStick = xbox.getLeftStickX(xbox.m_secondary);
        double err = 100;
        if (Math.abs(turretStick) > 0.25) {
          manager.safeRotateTurret(
              turret.getSpecificAngle() + (((turretStick - 0.25) / 0.75) * turretSpeed));
        } else {
          err = turnTurretToCam();
        }

        if (Math.abs(err) < 10) {
          if (cam.getLastReading().dist < 40) {
            LEDController.getInstance().setColor(Color.Green);
          } else {
            LEDController.getInstance().setColor(Color.Purple);
          }
          if (xbox.getRightTrigger(xbox.m_secondary)) {
            stage = Stage.Angling;
            armSetpoints =
                arm.calcFromCamera(cam.getLastReading().dist, height); // 54 for ship 36 for rocket
            LEDController.getInstance().setColor(Color.Default);
          }
        } else {
          LEDController.getInstance().setColor(Color.Default);
        }

        break;

      case Angling:
        drive.setPowers(0, 0);
        manager.safeAngleArm(armSetpoints[1]);
        if (Math.abs(arm.getAngle() - armSetpoints[1]) < 5 || arm.getAngle() > 45) {
          stage = Stage.Moving;
        }
        break;

      case Moving:
        drive.setPowers(0, 0);
        manager.safeExtendArm(armSetpoints[0] + (slowShoot ? 2 : 0));
        manager.safeAngleArm(armSetpoints[1]);
        if ((Math.abs(arm.getLength() - (armSetpoints[0] + (slowShoot ? 2 : 0))) < 1
                || arm.getLength() > 13)
            && Math.abs(arm.getAngle() - armSetpoints[1]) < 5) {
          lineUpCycles++;
        } else {
          lineUpCycles = 0;
        }
        if (lineUpCycles > 5) {
          stage = Stage.Scoring;
          scoringStart = Timer.getFPGATimestamp() * 1000;
        }
        break;

      case Scoring:
        if (slowShoot) {
          manip.intakeOutSlow();
        } else {
          manip.intakeOut();
        }
        if ((Timer.getFPGATimestamp() * 1000) - scoringStart > shootTime) {
          manip.intakeOff();
          stage = Stage.Resetting;
        }
        break;

      case Resetting:
        done();
        manager.resetFieldTurretAngle();
        stage = Stage.Done;
        break;

      case Done:
        break;
    }
  }

  @Override
  public void done() {
    cam.ledOff();
  }

  @Override
  public boolean isFinished() {
    return stage == Stage.Done;
  }

  private double turnTurretToCam() {
    cam.ledOn();
    CameraFrame read = cam.getLastReading();
    if (cam.targetFound() && read.tilt == LastTiltRead.Both) {
      double errX = read.x - (cam.cameraWidthPixels / 2);
      double errDegrees = ((errX / (cam.cameraWidthPixels / 2)) * (cam.cameraFOV / 2)) % 360;
      manager.safeRotateTurret(turret.getSpecificAngle() + errDegrees);
      return errDegrees;
    }
    return 100;
  }
}
