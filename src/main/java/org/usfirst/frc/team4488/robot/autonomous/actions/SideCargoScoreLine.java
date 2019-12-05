package org.usfirst.frc.team4488.robot.autonomous.actions;

import JavaRoboticsLib.Drive.DriveHelper;
import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team4488.robot.Constants;
import org.usfirst.frc.team4488.robot.operator.Controllers;
import org.usfirst.frc.team4488.robot.operator.Logging;
import org.usfirst.frc.team4488.robot.systems.Arm;
import org.usfirst.frc.team4488.robot.systems.Drive;
import org.usfirst.frc.team4488.robot.systems.Drive.LineSensors;
import org.usfirst.frc.team4488.robot.systems.Manipulator;
import org.usfirst.frc.team4488.robot.systems.SubsystemManager;
import org.usfirst.frc.team4488.robot.systems.Turret;

public class SideCargoScoreLine implements Action {

  private static SideCargoScoreLine inst;

  private enum Stage {
    Waiting,
    Scoring,
    Resetting,
    Done
  }

  private DriveHelper driveHelper;
  private SubsystemManager manager = SubsystemManager.getInstance();
  private Controllers xbox = Controllers.getInstance();
  private Drive drive = Drive.getInstance();
  private Turret turret = Turret.getInstance();
  private Manipulator manip = Manipulator.getInstance();
  private Arm arm = Arm.getInstance();
  private Stage stage = Stage.Waiting;
  private double startScoreTime = 0;
  private Logging logger = Logging.getInstance();

  private final double shootTime = 0.38;
  private final double turretShipCompensationScale = 45;
  private final double turretRocketCompensationScale = 15;
  private final double frontShipAngleCompensation = 5;
  private final double angleAdj = 5;

  private boolean finishedInitialTurret = false;

  public enum Targets {
    LeftShip,
    RightShip,
    FrontLeftShip,
    FrontRightShip,
    LeftRocket,
    RightRocket
  }

  private Targets target = Targets.LeftShip;

  public SideCargoScoreLine(Targets target) {
    driveHelper = new DriveHelper(drive, 0.1, 0.15);
    this.target = target;
    logger.writeToLogFormatted(this, target.name());
    logger.writeToLogFormatted(this, stage.name());
  }

  public static void teleRestart() {
    inst = new SideCargoScoreLine(Targets.FrontLeftShip);
    inst.start();
  }

  public static void teleUpdate() {
    inst.update();
  }

  public static boolean teleIsFinished() {
    return inst.isFinished();
  }

  public static void teleDone() {
    inst.done();
  }

  public static void setTarget(Targets target) {
    if (inst.target != target) {
      Logging.getInstance()
          .writeToLogFormatted(
              inst, target.name()); // cannot reference to "logger" because method is static
    }
    inst.target = target;
  }

  @Override
  public void start() {}

  @Override
  public void update() {
    switch (stage) {
      case Waiting:
        manager.driveUpdate();

        double fieldAngle;
        if (target == Targets.LeftShip || target == Targets.RightRocket) {
          fieldAngle = 90;
        } else if (target == Targets.RightShip || target == Targets.LeftRocket) {
          fieldAngle = 270;
        } else {
          fieldAngle = 0; // Front ship
        }

        // Decide which direction to compensate the turret angle in
        boolean reversed =
            (target == Targets.FrontLeftShip || target == Targets.FrontRightShip)
                ? (drive.getAngle() < 0)
                : (drive.getAngle() < -90 || drive.getAngle() > 90);
        boolean turretLeft = fieldAngle > 180 || fieldAngle == 0;
        boolean backwards = drive.getLinearSpeed() < 0;
        boolean direction = reversed ^ turretLeft ^ backwards;
        double scale =
            (target == Targets.LeftRocket || target == Targets.RightRocket)
                ? turretRocketCompensationScale
                : turretShipCompensationScale;
        double adjustedFieldAngle = fieldAngle;

        if (target == Targets.LeftShip || target == Targets.RightShip) {
          if (drive.getLinearSpeed() > 1) {
            if (direction) {
              adjustedFieldAngle -= angleAdj;
            } else {
              adjustedFieldAngle += angleAdj;
            }
          }
        }

        if (target == Targets.FrontLeftShip) {
          adjustedFieldAngle += frontShipAngleCompensation;
        } else if (target == Targets.FrontRightShip) {
          adjustedFieldAngle -= frontShipAngleCompensation;
        }

        turnTurretToFieldAngle(adjustedFieldAngle);

        // Decide which line sensor to use
        boolean forward = drive.getLinearSpeed() > 0;
        boolean leftSensor = turretLeft ^ reversed;
        LineSensors sensor;
        String sensorName;
        if (forward) {
          if (leftSensor) {
            sensor = LineSensors.LeftForward;
            sensorName = "LeftForward";
          } else {
            sensor = LineSensors.RightForward;
            sensorName = "RightForward";
          }
        } else {
          if (leftSensor) {
            sensor = LineSensors.LeftBack;
            sensorName = "LeftBack";
          } else {
            sensor = LineSensors.RightBack;
            sensorName = "RightBack";
          }
        }

        if (target == Targets.LeftRocket || target == Targets.RightRocket) {
          manager.safeAngleArm(Constants.rocketScoreAngle);
          manager.safeExtendArm(Constants.rocketScoreExtension);
        } else {
          manager.safeAngleArm(Constants.cargoShipScoreAngle);
          manager.safeExtendArm(Constants.cargoShipScoreExtension);
        }

        if (xbox.getRightTrigger(xbox.m_secondary) && drive.seesLine(sensor)) {
          logger.writeToLogFormatted(this, "Score triggered by " + sensorName + " sensor");
          startScoreTime = Timer.getFPGATimestamp();
          stage = Stage.Scoring;
          logger.writeToLogFormatted(this, stage.name());
          drive.setPowers(0, 0);
        }
        if (xbox.getB(xbox.m_secondary)) {
          startScoreTime = Timer.getFPGATimestamp();
          stage = Stage.Scoring;
          logger.writeToLogFormatted(this, stage.name());
          drive.setPowers(0, 0);
        }
        break;

      case Scoring:
        if (target == Targets.FrontLeftShip
            || target == Targets.FrontRightShip
            || target == Targets.LeftRocket
            || target == Targets.RightRocket) {
          manip.intakeOutSlow();
        } else {
          manip.intakeOut();
        }
        if (Timer.getFPGATimestamp() - startScoreTime > shootTime) {
          manip.intakeOff();
          stage = Stage.Resetting;
          logger.writeToLogFormatted(this, stage.name());
        }
        break;

      case Resetting:
        manager.resetFieldTurretAngle();
        stage = Stage.Done;
        logger.writeToLogFormatted(this, stage.name());
        break;

      case Done:
        break;
    }
  }

  @Override
  public boolean isFinished() {
    return stage == Stage.Done;
  }

  @Override
  public void done() {}

  private double turnTurretToFieldAngle(double fieldAngle) {
    double driveAngle = drive.getAngle();
    if (driveAngle < 0) driveAngle = 360 - Math.abs(driveAngle);
    double goalAngle = fieldAngle - driveAngle;
    while (goalAngle < 0) goalAngle += 360;

    double inverseAngle = goalAngle - 360;

    // -90 to 90 goal has no possible inverse
    if (goalAngle < 90) {
      manager.safeRotateTurret(goalAngle);
      return goalAngle;
    }

    if (goalAngle > 264 || goalAngle < -264) {
      manager.safeRotateTurret(inverseAngle);
      return inverseAngle;
    }

    if (inverseAngle > 264 || inverseAngle < -264) {
      manager.safeRotateTurret(goalAngle);
      return goalAngle;
    }

    double turretSpecificAngle = turret.getSpecificAngle();
    if (Math.abs(turretSpecificAngle - goalAngle) < Math.abs(turretSpecificAngle - inverseAngle)) {
      manager.safeRotateTurret(goalAngle);
      return goalAngle;
    } else {
      manager.safeRotateTurret(inverseAngle);
      return inverseAngle;
    }
  }
}
