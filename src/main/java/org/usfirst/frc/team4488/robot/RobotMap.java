package org.usfirst.frc.team4488.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import org.usfirst.frc.team4488.lib.util.PreferenceDoesNotExistException;
import org.usfirst.frc.team4488.lib.util.PreferencesParser;
import org.usfirst.frc.team4488.robot.systems.SpiderBotClimb;
import org.usfirst.frc.team4488.robot.systems.Subsystem;
import org.usfirst.frc.team4488.robot.systems.drive.Drive;
import org.usfirst.frc.team4488.robot.systems.drive.DummyDrive;
import org.usfirst.frc.team4488.robot.systems.drive.SwerveDrive;
import org.usfirst.frc.team4488.robot.systems.drive.SwerveModule;
import org.usfirst.frc.team4488.robot.systems.drive.SwerveModule.SwerveParameters;

public class RobotMap {
  // should never change this line
  public static RobotName robotName;

  public static final int PDP = 8;

  // talons
  public static final int DriveMotorRightM = 0;
  public static final int DriveMotorRight2 = 1;
  public static final int DriveMotorRight3 = 2;
  public static final int DriveMotorLeft3 = 3;
  public static final int DriveMotorLeft2 = 4;
  public static final int DriveMotorLeftM = 5;
  public static final int ClimberVacuumMotor = 0;

  // swerve
  public static final SwerveParameters[] swerveParameters =
      new SwerveParameters[] {
        // throttle id, angle id, pot id, pot offset
        new SwerveModule.SwerveParameters(5, 4, 2, 2746), // 0 front left
        new SwerveModule.SwerveParameters(3, 2, 1, 2952), // 1 front right
        new SwerveModule.SwerveParameters(7, 6, 3, 3964), // 2 back left
        new SwerveModule.SwerveParameters(1, 10, 0, 2186) // 3 back right
      };

  // solenoids
  public static final int DriveGearShiftSolenoid = 4;
  public static final int ClimberSolenoid = 0;

  // analog sensors
  public static final int ClimberPressureSensor = 0;

  // servos
  public static final int ClimberServo = 1;

  // DIOs

  // logic to determine which features to enable
  public static boolean driveExists = false;
  public static RobotSystems systems;

  private static String roboNameKey = "RobotName";

  public static RobotSystems robotSelector() {
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
      } else if (name.equals("SwervePlatform")) {
        robotName = RobotName.SwervePlatform;
      } else if (name.equals("MockPlatform")) {
        robotName = RobotName.MockPlatform;
      } else {
        robotName = RobotName.BareRoboRIO;
      }
    } catch (PreferenceDoesNotExistException e) {
      robotName = RobotName.BareRoboRIO;
    }

    ArrayList<Subsystem> otherSystems = new ArrayList<Subsystem>();
    switch (RobotMap.robotName) {
      case BareRoboRIO:
        driveExists = true;
        systems = new RobotSystems(DummyDrive.getInstance(), otherSystems);
        SmartDashboard.putString(roboNameKey, "BareRoboRIO");
        break;

      case ProgrammingPlatform:
        driveExists = true;
        systems = new RobotSystems(Drive.getInstance(), otherSystems);
        SmartDashboard.putString(roboNameKey, "ProgrammingPlatform");
        break;

      case SwervePlatform:
        driveExists = true;
        systems = new RobotSystems(SwerveDrive.getInstance(), otherSystems);
        SmartDashboard.putString(roboNameKey, "SwervePlatform");
        break;

      case MockPlatform:
        driveExists = true;
        otherSystems.add(SpiderBotClimb.getInstance());
        systems = new RobotSystems(Drive.getInstance(), otherSystems);
        SmartDashboard.putString(roboNameKey, "MockPlatform");
        break;

      case Competition:
      case Practice:
        driveExists = true;
        systems = new RobotSystems(Drive.getInstance(), otherSystems);
        SmartDashboard.putString(roboNameKey, "Practice/Competition");
        break;
    }

    return systems;
  }
}
