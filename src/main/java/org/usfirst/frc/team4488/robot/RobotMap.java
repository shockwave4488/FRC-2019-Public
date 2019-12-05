package org.usfirst.frc.team4488.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Arrays;
import java.util.List;
import org.usfirst.frc.team4488.lib.util.PreferenceDoesNotExistException;
import org.usfirst.frc.team4488.lib.util.PreferencesParser;
import org.usfirst.frc.team4488.robot.systems.Arm;
import org.usfirst.frc.team4488.robot.systems.Climber;
import org.usfirst.frc.team4488.robot.systems.Drive;
import org.usfirst.frc.team4488.robot.systems.Manipulator;
import org.usfirst.frc.team4488.robot.systems.Subsystem;
import org.usfirst.frc.team4488.robot.systems.Turret;

public class RobotMap {
  // should never change this line
  public static RobotName robotName;

  // talons
  public static final int DriveMotorRightM = 0;
  public static final int DriveMotorRight2 = 1;
  public static final int DriveMotorRight3 = 2;
  public static final int DriveMotorLeft3 = 3;
  public static final int DriveMotorLeft2 = 4;
  public static final int DriveMotorLeftM = 5;
  public static final int ClimberMaster = 6;
  public static final int ClimberSlave = 7;
  public static final int Intake = 10;
  public static final int TurretMotor = 11;
  public static final int FloorIntake = 12;

  public static final int armAngler = 8;
  public static final int armExtender = 9;

  // solenoids
  public static final int PanelGrabber = 1;
  public static final int DriveGearShiftSolenoid = 4;
  public static final int ClimberPancake = 0;

  // analog sensors
  public static final int driveBounceBack = 0;

  // DIOs
  public static final int CargoSensor = 5;
  public static final int TopJevoisLED = 0;
  public static final int LeftForwardLineSensor = 13;
  public static final int LeftBackLineSensor = 18;
  public static final int RightForwardLineSensor = 11;
  public static final int RightBackLineSensor = 12;

  // logic to determine which features to enable
  public static boolean driveExists = false;
  public static List<Subsystem> subsystemsToUse;

  private static String roboNameKey = "RobotName";

  public static List<Subsystem> robotSelector() {
    try {
      String name = PreferencesParser.getInstance().getString("RobotName");
      if (name.equals("Competition")) {
        robotName = RobotName.Competition;
      } else if (name.equals("Practice")) {
        robotName = RobotName.Practice;
      } else if (name.equals("BareRio")) {
        robotName = RobotName.BareRoboRIO;
      } else if (name.equals("ProgrammingPlatform")) {
        robotName = RobotName.ProgrammingPlatform;
      } else {
        robotName = RobotName.BareRoboRIO;
      }
    } catch (PreferenceDoesNotExistException e) {
      robotName = RobotName.BareRoboRIO;
    }

    switch (RobotMap.robotName) {
      case BareRoboRIO:
        driveExists = false;
        subsystemsToUse = Arrays.asList();
        SmartDashboard.putString(roboNameKey, "BareRoboRIO");
        break;

      case ProgrammingPlatform:
        driveExists = true;
        subsystemsToUse = Arrays.asList(Drive.getInstance());
        SmartDashboard.putString(roboNameKey, "ProgrammingPlatform");
        break;

      case Competition:
      case Practice:
        driveExists = true;
        subsystemsToUse =
            Arrays.asList(
                Drive.getInstance(),
                Turret.getInstance(),
                Climber.getInstance(),
                Manipulator.getInstance(),
                Arm.getInstance());

        SmartDashboard.putString(roboNameKey, "Practice/Competition");
        break;
    }

    return subsystemsToUse;
  }
}
