package org.usfirst.frc.team4488.robot.systems;

import JavaRoboticsLib.Drive.DriveHelper;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;
import org.usfirst.frc.team4488.robot.Constants;
import org.usfirst.frc.team4488.robot.Robot;
import org.usfirst.frc.team4488.robot.RobotMap;
import org.usfirst.frc.team4488.robot.autonomous.actions.Action;
import org.usfirst.frc.team4488.robot.autonomous.actions.CapturePanel;
import org.usfirst.frc.team4488.robot.autonomous.actions.LeftSandstorm;
import org.usfirst.frc.team4488.robot.autonomous.actions.LinePanelScore;
import org.usfirst.frc.team4488.robot.autonomous.actions.RightSandstorm;
import org.usfirst.frc.team4488.robot.autonomous.actions.SideCargoScore;
import org.usfirst.frc.team4488.robot.autonomous.actions.SideCargoScoreLine;
import org.usfirst.frc.team4488.robot.autonomous.actions.SideCargoScoreLine.Targets;
import org.usfirst.frc.team4488.robot.autonomous.actions.SidePanelScore;
import org.usfirst.frc.team4488.robot.loops.Looper;
import org.usfirst.frc.team4488.robot.operator.ButtonBox;
import org.usfirst.frc.team4488.robot.operator.Controllers;
import org.usfirst.frc.team4488.robot.operator.Controllers.SecondaryState;
import org.usfirst.frc.team4488.robot.operator.Logging;
import org.usfirst.frc.team4488.robot.systems.Arm.ArmAngleState;
import org.usfirst.frc.team4488.robot.systems.Arm.ArmExtendState;
import org.usfirst.frc.team4488.robot.systems.Camera.CameraFrame;
import org.usfirst.frc.team4488.robot.systems.Camera.LastTiltRead;
import org.usfirst.frc.team4488.robot.systems.LEDController.Color;
import org.usfirst.frc.team4488.robot.systems.Turret.TurretSafeState;

// import org.usfirst.frc.team4488.robot.systems.LEDController.Color;

/** Used to reset, start, stop, and update all subsystems at once */
public class SubsystemManager {

  private static SubsystemManager sInstance;

  private Controllers xbox = Controllers.getInstance();

  private final List<Subsystem> mAllSubsystems;

  private Drive drive = Drive.getInstance();
  private DriveHelper driveHelper;
  private SmartDrive smartDrive;

  private TurretSafeState turretSafe;
  private ArmAngleState armAngleState;
  private ArmExtendState armExtendState;

  private TurretSafeState turretSafeDes;
  private ArmAngleState armAngleStateDes;
  private ArmExtendState armExtendStateDes;

  private TurretSafeState proposedTurretSafe;
  private ArmExtendState proposedArmExtSafe;
  private ArmAngleState proposedArmAngSafe;

  private boolean xPressed = false;
  private boolean aPressed = false;
  private boolean bPressed = false;
  private boolean yPressed = false;
  private boolean leftTriggerPressed = false;
  private boolean primaryTriggersPressed = false;

  private boolean atExtend = false;

  private boolean rightTriggerPressed;

  private boolean armMoveTo;

  private double turretTarget;

  private int cargoStage = 0;

  private int climbStage = 0;

  private double climbLineUp;

  private boolean isSafe;

  private Targets lastTargetSelected = Targets.LeftShip;

  private Turret turret = Turret.getInstance();
  private Manipulator manipulator = Manipulator.getInstance();
  private Arm arm = Arm.getInstance();
  private ButtonBox box = ButtonBox.getInstance();
  private double turretAngleGoal = 0;

  private double turretProposedPos;
  private double armProposedExt;
  private double armProposedAng;

  public enum CargoPickupStage {
    TurretDirection,
    ArmOut,
    ArmDown,
    WaitForCargo,
    ArmAlive,
    ArmUp,
    ArmIn
  }

  private double armAngBox;
  private double armExtBox;
  private double turAngBox;

  public enum RobotStates {
    Manual,
    Climb,
    LoadingStationPanelPickup,
    ScorePanel,
    CapturePanel,
    FloorCargoPickup,
    FloorPanelPickup,
    ScoringCargoShip,
    SideCargoScoring,
    SidePanelScoring,
    LinePanelScoring,
    AutoDrive,
    ButtonBoxArmPos,
    Sandstorm
  }

  private RobotStates state;
  private CargoPickupStage cargoPickupStage;
  private Action sandstormAction;
  private SendableChooser<Sides> sideSelector = new SendableChooser<Sides>();

  public enum Sides {
    Blue,
    Red
  }

  public static SubsystemManager getInstance() {
    if (sInstance == null) {
      sInstance = new SubsystemManager(RobotMap.robotSelector());
    }

    return sInstance;
  }

  public SubsystemManager(List<Subsystem> allSubsystems) {
    mAllSubsystems = allSubsystems;
    driveHelper = new DriveHelper(drive, 0.1, 0.15); // throttle deadzone, turn deadzone
    smartDrive = new SmartDrive(drive);
    state = RobotStates.Manual;
    cargoPickupStage = CargoPickupStage.TurretDirection;

    sideSelector.addDefault("Blue", Sides.Blue);
    sideSelector.addObject("Red", Sides.Red);
    SmartDashboard.putData("SideSelector", sideSelector);
  }

  public void updateSmartDashboard() {
    mAllSubsystems.forEach((s) -> s.updateSmartDashboard());
  }

  public void writeToLog() {
    mAllSubsystems.forEach((s) -> s.writeToLog());
  }

  public void stop() {
    mAllSubsystems.forEach((s) -> s.stop());
    Camera.getInstance().ledOff();
    // LEDController.getInstance().setColor(Color.Default);
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
    ButtonBox box = ButtonBox.getInstance();
    if (box.getLShip1()) {
      lastTargetSelected = Targets.FrontLeftShip;
    } else if (box.getRShip1()) {
      lastTargetSelected = Targets.FrontRightShip;
    } else if (box.getLShip2() || box.getLShip3() || box.getLShip4()) {
      lastTargetSelected = Targets.LeftShip;
    } else if (box.getRShip2() || box.getRShip3() || box.getRShip4()) {
      lastTargetSelected = Targets.RightShip;
    } else if (box.getLRocketC() || box.getLRocketP1() || box.getLRocketP2()) {
      lastTargetSelected = Targets.LeftRocket;
    } else if (box.getRRocketC() || box.getRRocketP1() || box.getLRocketP2()) {
      lastTargetSelected = Targets.RightRocket;
    }

    if (xbox.getSelect(xbox.m_secondary)) {
      state = RobotStates.Manual;
    }

    switch (state) {
      case Manual:
        manualUpdate();
        break;
      case Climb:
        climbUpdate();
        break;
      case FloorCargoPickup:
        floorCargoPickupUpdate();
        break;
      case CapturePanel:
        capturePanelUpdate();
        break;
      case ScorePanel:
        scorePanelUpdate();
        break;
      case SideCargoScoring:
        sideCargoScoreLineUpdate();
        break;
      case SidePanelScoring:
        sidePanelScoreUpdate();
        break;
      case ButtonBoxArmPos:
        buttonBoxArmPosUpdate();
        break;
      case Sandstorm:
        sandstormUpdate();
        break;
      case LinePanelScoring:
        linePanelScoringUpdate();
        break;
      default:
        break;
    }
  }

  private void manualUpdate() {

    // Manual and assisted controls

    Camera.getInstance().ledOff();

    if (xbox.getStart(xbox.m_secondary)) {
      xbox.secondaryState = SecondaryState.auto;
      Logging.getInstance().writeToLogFormatted(SecondaryState.class, xbox.secondaryState.name());
      resetFieldTurretAngle();
    } else if (xbox.getSelect(xbox.m_secondary)) {
      xbox.secondaryState = SecondaryState.manual;
      Logging.getInstance().writeToLogFormatted(SecondaryState.class, xbox.secondaryState.name());
    }

    driveUpdate();

    if (box.getLShip2() && Robot.isAuto) {
      state = RobotStates.Sandstorm;
      sandstormAction = new LeftSandstorm();
      sandstormAction.start();
    } else if (box.getRShip2() && Robot.isAuto) {
        state = RobotStates.Sandstorm;
        sandstormAction = new RightSandstorm();
        sandstormAction.start();
    }

    if (xbox.getLeftBumper(xbox.m_secondary)) {
      armProposedExt = Arm.getInstance().getDesiredLength() - 0.4;
      safeExtendArm(armProposedExt);
    }

    if (xbox.getRightBumper(xbox.m_secondary)) {
      armProposedExt = Arm.getInstance().getDesiredLength() + 0.4;
      safeExtendArm(armProposedExt);
    }

    if (((xbox.getDPad(xbox.m_primary) >= 315 || xbox.getDPad(xbox.m_primary) <= 45)
            && xbox.getDPad(xbox.m_primary) >= 0)
        && ((xbox.getDPad(xbox.m_secondary) >= 315 || xbox.getDPad(xbox.m_secondary) <= 45)
            && xbox.getDPad(xbox.m_secondary) >= 0)) {
      LEDController.getInstance().setColor(Color.Rainbow);
      climbLineUp = drive.getLinearDistance() - Constants.climbLineUp;
      state = RobotStates.Climb;
      Logging.getInstance().writeToLogFormatted(RobotStates.class, state.name());
      climbStage = 0;
    }

    if (xbox.secondaryState == SecondaryState.auto) {

      // Auto Controls

      if (Math.abs(xbox.getRightStickY(xbox.m_secondary)) > 0.25) {
        armProposedAng =
            Arm.getInstance().getDesiredAngle() + (1 * xbox.getRightStickY(xbox.m_secondary));
        safeAngleArm(armProposedAng);
      }
  
      if (Math.abs(xbox.getRightStickY(xbox.m_secondary)) > 0.25) {
        armProposedAng =
            Arm.getInstance().getDesiredAngle() + (2 * xbox.getRightStickY(xbox.m_secondary));
        safeAngleArm(armProposedAng);
      }

      turretUpdate();

      if (xbox.getY(xbox.m_secondary) && !yPressed) {
        state = RobotStates.LinePanelScoring;
        LinePanelScore.teleRestart();
      }
      yPressed = xbox.getY(xbox.m_secondary);

      if (xbox.getRightTrigger(xbox.m_primary)) {
        safeExtendArm(Constants.armExtendLowMid1 + 0.1);
        safeRotateTurret(0);
        primaryTriggersPressed = true;
      } else if (xbox.getLeftTrigger(xbox.m_primary)) {
        safeExtendArm(Constants.armExtendLowMid1 + 0.1);
        if (turret.getSpecificAngle() < 0) {
          safeRotateTurret(-180);
        } else {
          safeRotateTurret(180);
        }
        primaryTriggersPressed = true;
      } else if (primaryTriggersPressed) {
        resetFieldTurretAngle();
        primaryTriggersPressed = false;
      }

      if (!rightTriggerPressed
          && xbox.getRightTrigger(xbox.m_secondary)
          && !manipulator.hasCargo()) {
        cargoStage = 0;
        state = RobotStates.FloorCargoPickup;
        Logging.getInstance().writeToLogFormatted(RobotStates.class, state.name());
      }
      rightTriggerPressed = xbox.getRightTrigger(xbox.m_secondary);

      if (xbox.getLeftTrigger(xbox.m_secondary) && !leftTriggerPressed) {
        state = RobotStates.SideCargoScoring;
        SideCargoScoreLine.teleRestart();
      }
      leftTriggerPressed = xbox.getLeftTrigger(xbox.m_secondary);

      if (xbox.getX(xbox.m_secondary) && !xPressed) {
        state = RobotStates.SidePanelScoring;
        Logging.getInstance().writeToLogFormatted(RobotStates.class, state.name());
        SidePanelScore.teleRestart(lastTargetSelected);
      }
      xPressed = xbox.getX(xbox.m_secondary);

      if (xbox.getA(xbox.m_secondary) && !aPressed) {
        state = RobotStates.CapturePanel;
        CapturePanel.teleRestart();
        Logging.getInstance().writeToLogFormatted(RobotStates.class, state.name());
      }
      aPressed = xbox.getA(xbox.m_secondary);

      /*if (xbox.getB(xbox.m_secondary) && !bPressed) {
        state = RobotStates.ScorePanel;
        Logging.getInstance().writeToLogFormatted(RobotStates.class, state.name());
      }*/
      // bPressed = xbox.getB(xbox.m_secondary);

      if (box.getLShip1() || box.getRShip1()) {
        if (manipulator.hasCargo()) {
          armAngBox = Constants.armAngleThreshMidHigh - 1; // value needs testing
        } else {
          armAngBox = (Constants.armAngleThreshMidHigh + Constants.armAngleThreshLowMid) / 2;
        }
        armExtBox = Constants.armExtendLowMid1 + 1;
        turAngBox = 0;
        // buttonBoxArmPosUpdate();
        // state = RobotStates.ButtonBoxArmPos;
        armMoveTo = true;
      }

      if (box.getLShip2() || box.getLShip3() || box.getLShip4()) {
        if (manipulator.hasCargo()) {
          armAngBox = Constants.armAngleThreshMidHigh - 1; // value needs testing
          armExtBox = Constants.armExtendLowMid1 + 1;
        } else {
          armAngBox = (Constants.armAngleThreshMidMid) + 2;
          armExtBox = 1;
        }
        turAngBox = 90;
        // buttonBoxArmPosUpdate();
        // state = RobotStates.ButtonBoxArmPos;
        armMoveTo = true;
      }

      if (box.getRShip2() || box.getRShip3() || box.getRShip4()) {
        if (manipulator.hasCargo()) {
          armAngBox = Constants.armAngleThreshMidHigh - 1; // value needs testing
          armExtBox = Constants.armExtendLowMid1 + 1;
        } else {
          armAngBox = Constants.armAngleThreshMidMid + 2;
          armExtBox = 1;
        }
        turAngBox = -90;
        // buttonBoxArmPosUpdate();
        // state = RobotStates.ButtonBoxArmPos;
        armMoveTo = true;
      }

      if (box.getLRocketC()) {
        armAngBox =
            (Constants.armAngleThreshMidHigh + Constants.armAngleThreshLowMid)
                / 2; // value needs testing
        armExtBox = Constants.armExtendLowMid1 + 1;
        turAngBox = -90;
        // buttonBoxArmPosUpdate();
        // state = RobotStates.ButtonBoxArmPos;
        armMoveTo = true;
      }

      if (box.getRRocketC()) {
        armAngBox =
            (Constants.armAngleThreshMidHigh + Constants.armAngleThreshLowMid)
                / 2; // value needs testing
        armExtBox = Constants.armExtendLowMid1 + 1;
        turAngBox = 90;
        // buttonBoxArmPosUpdate();
        armMoveTo = true;
      }

      if (box.getHumanP1() || box.getHumanP2()) {
        armAngBox =
            (Constants.armAngleThreshMidHigh + Constants.armAngleThreshLowMid)
                / 2; // value needs testing
        armExtBox = Constants.armExtendLowMid1 + 1;
        turAngBox = 180;
        armMoveTo = true;
      }

      if (box.getLRocketP1()) {
        armAngBox =
            (Constants.armAngleThreshMidHigh + Constants.armAngleThreshLowMid)
                / 2; // value needs testing
        armExtBox = Constants.armExtendLowMid1 + 1;
        turAngBox = -27;
        armMoveTo = true;
      }

      if (box.getLRocketP2()) {
        armAngBox =
            (Constants.armAngleThreshMidHigh + Constants.armAngleThreshLowMid)
                / 2; // value needs testing
        armExtBox = Constants.armExtendLowMid1 + 1;
        turAngBox = -153;
        armMoveTo = true;
      }

      if (box.getRRocketP1()) {
        armAngBox =
            (Constants.armAngleThreshMidHigh + Constants.armAngleThreshLowMid)
                / 2; // value needs testing
        armExtBox = Constants.armExtendLowMid1 + 1;
        turAngBox = 27;
        armMoveTo = true;
      }

      if (box.getRRocketP2()) {
        armAngBox =
            (Constants.armAngleThreshMidHigh + Constants.armAngleThreshLowMid)
                / 2; // value needs testing
        armExtBox = Constants.armExtendLowMid1 + 1;
        turAngBox = 153;
        armMoveTo = true;
      }

      if (xbox.getB(xbox.m_secondary)) {
        manipulator.intakeOut();
      } else {
        manipulator.intakeOff();
      }

    } else {

      // Manual Controls

      if (Math.abs(xbox.getRightStickY(xbox.m_secondary)) > 0.25) {
        armProposedAng =
            Arm.getInstance().getDesiredAngle() + (1 * xbox.getRightStickY(xbox.m_secondary));
        Arm.getInstance().setAngle(armProposedAng);
      }
  
      if (Math.abs(xbox.getRightStickY(xbox.m_secondary)) > 0.25) {
        armProposedAng =
            Arm.getInstance().getDesiredAngle() + (2 * xbox.getRightStickY(xbox.m_secondary));
        Arm.getInstance().setAngle(armProposedAng);
      }

      if (Math.abs(xbox.getLeftStickX(xbox.m_secondary)) > 0.2) {
        safeRotateTurret(turret.getSpecificAngle() + (xbox.getLeftStickX(xbox.m_secondary) * 15));
      }

      if (xbox.getX(xbox.m_secondary) && !xPressed) {
        if (Manipulator.getInstance().isHoldingPanel()) {
          Manipulator.getInstance().releasePanel();
        } else {
          Manipulator.getInstance().holdPanel();
        }
      }
      xPressed = xbox.getX(xbox.m_secondary);

      if (xbox.getLeftTrigger(xbox.m_secondary)) {
        manipulator.intakeOut();
      } else if (xbox.getRightTrigger(xbox.m_secondary)) {
        manipulator.intakeIn();
      } else {
        manipulator.intakeOff();
      }

      if (xbox.getA(xbox.m_secondary) && xbox.getB(xbox.m_secondary)) {
        turret.zeroAtCurrent();
      }
    }

    turretSafeDes = Turret.getInstance().safeState(Turret.getInstance().getTargetAngle());
    armAngleStateDes = Arm.getInstance().angSafeState(Arm.getInstance().getDesiredAngle());
    armExtendStateDes = Arm.getInstance().extSafeState(Arm.getInstance().getDesiredLength());

    turretSafe = Turret.getInstance().safeState(Turret.getInstance().getSpecificAngle());
    armAngleState = Arm.getInstance().angSafeState(Arm.getInstance().getAngle());
    armExtendState = Arm.getInstance().extSafeState(Arm.getInstance().getLength());

    if (armMoveTo) {
      buttonBoxArmPosUpdate();
    }
  }

  private void buttonBoxArmPosUpdate() {
    safeAngleArm(armAngBox);
    safeExtendArm(armExtBox);
    turretAngleGoal = turAngBox;

    if (xbox.getLeftBumper(xbox.m_secondary)
        || xbox.getRightBumper(xbox.m_secondary)
        || xbox.getA(xbox.m_secondary)
        || xbox.getB(xbox.m_secondary)
        || xbox.getDPadPressed(xbox.m_secondary)
        || xbox.getLeftStickPress(xbox.m_secondary)
        || xbox.getLeftTrigger(xbox.m_secondary)
        || xbox.getRightStickPress(xbox.m_secondary)
        || xbox.getRightTrigger(xbox.m_secondary)
        || xbox.getSelect(xbox.m_secondary)
        || xbox.getStart(xbox.m_secondary)
        || xbox.getX(xbox.m_secondary)
        || xbox.getY(xbox.m_secondary)
        || xbox.getB(xbox.m_secondary)
        || xbox.getDPadPressed(xbox.m_secondary)
        || xbox.getLeftStickPress(xbox.m_secondary)
        || xbox.getLeftTrigger(xbox.m_secondary)
        || xbox.getRightStickPress(xbox.m_secondary)
        || xbox.getRightTrigger(xbox.m_secondary)
        || xbox.getSelect(xbox.m_secondary)
        || xbox.getStart(xbox.m_secondary)
        || xbox.getX(xbox.m_secondary)
        || xbox.getY(xbox.m_secondary)
        || xbox.getDPad(xbox.m_secondary) > -1
        || Math.abs(xbox.getLeftStickX(xbox.m_secondary)) > 0.1
        || Math.abs(xbox.getLeftStickY(xbox.m_secondary)) > 0.1
        || Math.abs(xbox.getRightStickX(xbox.m_secondary)) > 0.1
        || Math.abs(xbox.getRightStickY(xbox.m_secondary)) > 0.1) {
      armMoveTo = false;
    }

    SmartDashboard.putBoolean(
        "is des safe",
        Constants.armSafety[turretSafeDes.val][armExtendStateDes.val][armAngleStateDes.val]);
  }

  private void linePanelScoringUpdate() {
    smartDrive.setLinearDoneRange(0.25);
    LinePanelScore.teleUpdate();
    if ((xbox.getY(xbox.m_secondary) && !yPressed) || LinePanelScore.teleIsFinished()) {
      resetFieldTurretAngle();
      smartDrive.setLinearDoneRange(0.5);
      state = RobotStates.Manual;
      Logging.getInstance().writeToLogFormatted(RobotStates.class, state.name());
    }
    yPressed = xbox.getY(xbox.m_secondary);
  }

  private void floorCargoPickupUpdate() {
    manipulator.holdPanel();
    driveUpdate();
    switch (cargoPickupStage) {
      case TurretDirection:
        safeAngleArm(5);
        if (turret.getSpecificAngle() >= -90 && turret.getSpecificAngle() <= 90) {
          safeRotateTurret(0);
          turretTarget = 0;
        } else if (turret.getSpecificAngle() < -90) {
          safeRotateTurret(-180);
          turretTarget = -180;
        } else if (turret.getSpecificAngle() > 90) {
          safeRotateTurret(180);
          turretTarget = 180;
        }
        if (Math.abs(turret.getSpecificAngle() - turretTarget) < 4) {
          cargoPickupStage = CargoPickupStage.ArmOut;
        }

        if (xbox.secondaryState == SecondaryState.auto && !xbox.getRightTrigger(xbox.m_secondary)) {
          cargoPickupStage = CargoPickupStage.TurretDirection;
          resetFieldTurretAngle();
          state = RobotStates.Manual;
        }
        break;
      case ArmOut:
        safeExtendArm(14);
        safeRotateTurret(turretTarget);

        if (arm.getLength() > 13) {
          cargoPickupStage = CargoPickupStage.ArmDown;
        }

        if (xbox.secondaryState == SecondaryState.auto && !xbox.getRightTrigger(xbox.m_secondary)) {
          cargoPickupStage = CargoPickupStage.ArmIn;
        }
        break;
      case ArmDown:
        manipulator.intakeIn();

        safeAngleArm(arm.getCargoIntakeAngle());

        if (Math.abs(arm.getAngle() - arm.getCargoIntakeAngle()) < 1) {
          cargoPickupStage = CargoPickupStage.WaitForCargo;
        }

        if (xbox.secondaryState == SecondaryState.auto && !xbox.getRightTrigger(xbox.m_secondary)) {
          manipulator.intakeOff();
          cargoPickupStage = CargoPickupStage.ArmAlive;
        }
        break;
      case WaitForCargo:
        manipulator.intakeIn();
        if (manipulator.hasCargo()) {
          cargoPickupStage = CargoPickupStage.ArmAlive;
          manipulator.intakeOff();
        }

        if (xbox.secondaryState == SecondaryState.auto && !xbox.getRightTrigger(xbox.m_secondary)) {
          manipulator.intakeOff();
          cargoPickupStage = CargoPickupStage.ArmAlive;
        }
        break;
      case ArmAlive:
        cargoPickupStage = CargoPickupStage.ArmUp;
        break;
      case ArmUp:
        manipulator.intakeOff();
        safeAngleArm(9);
        if (arm.getAngle() < 12 && arm.getAngle() > 6) {
          cargoPickupStage = CargoPickupStage.ArmIn;
        }
        break;
      case ArmIn:
        safeExtendArm(6);
        if (arm.getLength() > 5 && arm.getLength() < 7) {
          cargoPickupStage = CargoPickupStage.TurretDirection;
          manipulator.intakeOff();
          resetFieldTurretAngle();
          state = RobotStates.Manual;
          Logging.getInstance().writeToLogFormatted(RobotStates.class, state.name());
        }
        break;
    }
  }

  public void driveUpdate() {
    if (RobotMap.driveExists) {
      double[] powers =
          drive.calcPowers(xbox.getLeftStickY(xbox.m_primary), xbox.getRightStickX(xbox.m_primary));
      if (xbox.getA(xbox.m_primary)) {
        drive.driveStraightField(powers[0]);
      } else if (xbox.getB(xbox.m_primary)) {
        double angle = 0;
        if (lastTargetSelected == Targets.LeftShip) {
          angle = 90;
        } else if (lastTargetSelected == Targets.RightShip) {
          angle = -90;
        }
        drive.driveToAngle(powers[0], angle);
      } else {
        driveHelper.Drive(powers[0], powers[1]);
      }
    }
  }

  private void climbUpdate() {
    switch (climbStage) {
      case 0:
        smartDrive.driveToDistance(climbLineUp);
        if (smartDrive.isDriveDistanceDone()) {
          drive.setPowers(0, 0);
          climbStage = 1;
          Logging.getInstance().writeToLogFormatted(Climber.class, "" + climbStage);
        }
        break;
      case 1:
        Climber.getInstance().deploy();
        if ((xbox.getDPad(xbox.m_primary) >= 135 && xbox.getDPad(xbox.m_primary) <= 225)
            || (xbox.getDPad(xbox.m_secondary) >= 135 && xbox.getDPad(xbox.m_secondary) <= 225)) {
          LEDController.getInstance().setColor(Color.Default);
          Climber.getInstance().stop();
          resetFieldTurretAngle();
          state = RobotStates.Manual;
          Logging.getInstance().writeToLogFormatted(RobotStates.class, state.name());
        }
        if (Climber.getInstance().getTicks() > Constants.climberDoneTicks - 1000) {
          safeExtendArm(13);
          safeAngleArm(0);
          turretUpdate();
          driveUpdate();
        }

        break;
    }
  }

  public void safeExtendArm(double armProposedExt) {
    double armCurrExt = Arm.getInstance().getLength();
    double armProposedExtAdj;

    if ((Math.abs(armProposedExt - armCurrExt)) > (Constants.safeDesDiffArmExt)) {
      if (armProposedExt > armCurrExt) {
        armProposedExtAdj = armCurrExt + Constants.safeDesDiffArmExt;
      } else {
        armProposedExtAdj = armCurrExt - Constants.safeDesDiffArmExt;
      }
    } else {
      armProposedExtAdj = armProposedExt;
    }

    turretSafeDes = Turret.getInstance().safeState(Turret.getInstance().getTargetAngle());
    armAngleStateDes = Arm.getInstance().angSafeState(Arm.getInstance().getDesiredAngle());

    proposedArmExtSafe = Arm.getInstance().extSafeState(armProposedExtAdj);
    isSafe = Constants.armSafety[turretSafeDes.val][proposedArmExtSafe.val][armAngleStateDes.val];

    if (isSafe) Arm.getInstance().setLength(armProposedExtAdj);
  }

  public void safeAngleArm(double armProposedAng) {
    double armCurrAng = Arm.getInstance().getAngle();
    double armProposedAngAdj;
    if ((Math.abs(armProposedAng - armCurrAng)) >= (Constants.safeDesDiffArmAng)) {
      if (armProposedAng > armCurrAng) {
        armProposedAngAdj = armCurrAng + Constants.safeDesDiffArmAng;
      } else {
        armProposedAngAdj = armCurrAng - Constants.safeDesDiffArmAng;
      }
    } else {
      armProposedAngAdj = armProposedAng;
    }

    turretSafeDes = Turret.getInstance().safeState(Turret.getInstance().getTargetAngle());
    armExtendStateDes = Arm.getInstance().extSafeState(Arm.getInstance().getDesiredLength());

    proposedArmAngSafe = Arm.getInstance().angSafeState(armProposedAngAdj);
    isSafe = Constants.armSafety[turretSafeDes.val][armExtendStateDes.val][proposedArmAngSafe.val];

    if (isSafe) Arm.getInstance().setAngle(armProposedAngAdj);
  }

  public void safeRotateTurret(double turretProposedPos) {
    double turretCurrPos = Turret.getInstance().getSpecificAngle();
    double turretProposedPosAdj;
    if ((Math.abs(turretProposedPos - turretCurrPos)) > (Constants.safeDesDiffTurret)) {
      if (turretProposedPos > turretCurrPos) {
        turretProposedPosAdj = turretCurrPos + Constants.safeDesDiffTurret;
      } else {
        turretProposedPosAdj = turretCurrPos - Constants.safeDesDiffTurret;
      }
    } else {
      turretProposedPosAdj = turretProposedPos;
    }

    armExtendStateDes = Arm.getInstance().extSafeState(Arm.getInstance().getDesiredLength());
    armAngleStateDes = Arm.getInstance().angSafeState(Arm.getInstance().getDesiredAngle());

    proposedTurretSafe = Turret.getInstance().safeState(turretProposedPosAdj);
    isSafe =
        Constants.armSafety[proposedTurretSafe.val][armExtendStateDes.val][armAngleStateDes.val];

    if (isSafe) {
      Turret.getInstance().goToAngle(turretProposedPosAdj);
    }
  }

  private void capturePanelUpdate() {
    if ((xbox.getA(xbox.m_secondary) && !aPressed) || CapturePanel.teleIsFinished()) {
      resetFieldTurretAngle();
      state = RobotStates.Manual;
      Logging.getInstance().writeToLogFormatted(RobotStates.class, state.name());
    }
    aPressed = xbox.getA(xbox.m_secondary);

    CapturePanel.teleUpdate();
  }

  private void scorePanelUpdate() {

    if (-90 < turret.getSpecificAngle() && turret.getSpecificAngle() < 90) turret.goToAngle(0);
    else if (turret.getSpecificAngle() < -90) turret.goToAngle(-180);
    else turret.goToAngle(180);

    safeExtendArm(12);
    safeAngleArm(-2);

    if (arm.getLength() > 11) {
      atExtend = true;
    }

    Camera cam = Camera.getInstance();
    cam.ledOn();
    SmartDashboard.putBoolean("target found", cam.targetFound());
    if (cam.targetFound()) {
      // LEDController.getInstance().setColor(Color.Green);
      double camError = cam.getLastReading().x - (cam.cameraWidthPixels / 2);
      double powerMod = (camError / (cam.cameraWidthPixels / 2)) * 0.4;
      double throttle = xbox.getLeftStickY(xbox.m_primary);
      drive.setPowers(throttle + powerMod, throttle - powerMod);
    } else {
      // LEDController.getInstance().setColor(Color.Purple);
      driveUpdate();
    }

    if (arm.isCurrentPlacementThresholdPassed() && atExtend) {
      manipulator.releasePanel();
      // LEDController.getInstance().setColor(Color.Default);
      resetFieldTurretAngle();
      state = RobotStates.Manual;
      Logging.getInstance().writeToLogFormatted(RobotStates.class, state.name());
      safeExtendArm(6);
    }

    if (xbox.getX(xbox.m_secondary) && !xPressed) {
      // LEDController.getInstance().setColor(Color.Default);
      state = RobotStates.Manual;
      Logging.getInstance().writeToLogFormatted(RobotStates.class, state.name());
      resetFieldTurretAngle();
      safeExtendArm(6);
      System.out.println("abort");
    }
    xPressed = xbox.getX(xbox.m_secondary);
  }

  private void visionUpdate() {
    Camera.getInstance().ledOn();
    SmartDashboard.putBoolean("Target Found", Camera.getInstance().targetFound());
    if (Camera.getInstance().targetFound()) {
      CameraFrame read = Camera.getInstance().getLastReading();
      double errX =
          Camera.getInstance().getLastReading().x - (Camera.getInstance().cameraWidthPixels / 2);
      if (read.tilt == LastTiltRead.Left) errX += 10;
      else if (read.tilt == LastTiltRead.Right) errX -= 10;
      double errDegrees =
          ((errX / (Camera.getInstance().cameraWidthPixels / 2))
                  * (Camera.getInstance().cameraFOV / 2))
              % 360;
      safeRotateTurret(Turret.getInstance().getSpecificAngle() + errDegrees);
    }
  }

  public void turretUpdate() {
    double stickX = xbox.getLeftStickX(xbox.m_secondary);
    double stickY = xbox.getLeftStickY(xbox.m_secondary);
    if (Math.sqrt(Math.pow(stickX, 2) + Math.pow(stickY, 2)) > 0.5) {
      double atan = Math.atan2(stickY, stickX);
      atan += Math.PI;
      atan = (atan / Math.PI) * 180 - 270;
      if (atan < 0) atan = 360 - Math.abs(atan);
      double stickAngle = 360 - atan;

      turretAngleGoal = stickAngle;
    }

    turnTurretToField(turretAngleGoal);
  }

  public void turnTurretToField(double fieldAngle) {
    double driveAngle = Drive.getInstance().getAngle();
    if (driveAngle < 0) driveAngle = 360 - Math.abs(driveAngle);
    double goalAngle = fieldAngle - driveAngle;
    while (goalAngle < 0) goalAngle += 360;

    double inverseAngle = goalAngle - 360;

    // -90 to 90 goal has no possible inverse
    if (goalAngle < 90) {
      safeRotateTurret(goalAngle);
      return;
    }

    if (goalAngle > 270 || goalAngle < -270) {
      safeRotateTurret(inverseAngle);
      return;
    }

    if (inverseAngle > 270 || inverseAngle < -270) {
      safeRotateTurret(goalAngle);
      return;
    }

    double turretSpecificAngle = Turret.getInstance().getSpecificAngle();
    if (Math.abs(turretSpecificAngle - goalAngle) < Math.abs(turretSpecificAngle - inverseAngle)) {
      safeRotateTurret(goalAngle);
    } else {
      safeRotateTurret(inverseAngle);
    }
  }

  private void sideCargoScoreUpdate() {
    SideCargoScore.teleUpdate();
    if ((xbox.getLeftTrigger(xbox.m_secondary) && !leftTriggerPressed)
        || SideCargoScore.teleIsFinished()) {
      resetFieldTurretAngle();
      state = RobotStates.Manual;
      Logging.getInstance().writeToLogFormatted(RobotStates.class, state.name());
    }
    leftTriggerPressed = xbox.getLeftTrigger(xbox.m_secondary);

    if (box.getLShip1()
        || box.getLShip2()
        || box.getLShip3()
        || box.getLShip4()
        || box.getRShip1()
        || box.getRShip2()
        || box.getRShip3()
        || box.getRShip4()) {
      SideCargoScore.setHeight(80);
      SideCargoScore.setSlowShoot(false);
      // LEDController.getInstance().blinkColor(Color.Purple);
    }
    if (box.getLRocketC() || box.getRRocketC()) {
      SideCargoScore.setHeight(40);
      SideCargoScore.setSlowShoot(true);
      // g.getInstance().blinkColor(Color.Purple);
    }
  }

  private void sideCargoScoreLineUpdate() {
    SideCargoScoreLine.teleUpdate();
    if ((xbox.getLeftTrigger(xbox.m_secondary) && !leftTriggerPressed)
        || SideCargoScoreLine.teleIsFinished()) {
      resetFieldTurretAngle();
      state = RobotStates.Manual;
    }
    leftTriggerPressed = xbox.getLeftTrigger(xbox.m_secondary);

    SideCargoScoreLine.setTarget(lastTargetSelected);
  }

  private void sidePanelScoreUpdate() {
    SidePanelScore.teleUpdate();
    SidePanelScore.setTarget(lastTargetSelected);
    if ((xbox.getX(xbox.m_secondary) && !xPressed) || SidePanelScore.teleIsFinished()) {
      resetFieldTurretAngle();
      state = RobotStates.Manual;
      Logging.getInstance().writeToLogFormatted(RobotStates.class, state.name());
    }
    xPressed = xbox.getX(xbox.m_secondary);
  }

  private void sandstormUpdate() {
    if (xbox.getB(xbox.m_secondary) || sandstormAction.isFinished()) {
      state = RobotStates.Manual;
      resetFieldTurretAngle();
      drive.configPercentVbus();
    }
    bPressed = xbox.getB(xbox.m_secondary);

    sandstormAction.update();
  }

  public void resetFieldTurretAngle() {
    turretAngleGoal = turret.getSpecificAngle() + drive.getAngle();
    if (turretAngleGoal < 0) turretAngleGoal += 360;
  }

  public Sides getSelectedSide() {
    return sideSelector.getSelected();
  }
}
