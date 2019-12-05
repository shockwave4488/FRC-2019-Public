package org.usfirst.frc.team4488.robot;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4488.lib.util.NavX;
import org.usfirst.frc.team4488.robot.app.RobotState;
import org.usfirst.frc.team4488.robot.autonomous.AutoModeExecuter;
import org.usfirst.frc.team4488.robot.autonomous.AutoModeSelector;
import org.usfirst.frc.team4488.robot.loops.Looper;
import org.usfirst.frc.team4488.robot.loops.RobotStateEstimator;
import org.usfirst.frc.team4488.robot.operator.Controllers;
import org.usfirst.frc.team4488.robot.operator.Logging;
import org.usfirst.frc.team4488.robot.systems.Arm;
import org.usfirst.frc.team4488.robot.systems.Camera;
import org.usfirst.frc.team4488.robot.systems.Climber;
import org.usfirst.frc.team4488.robot.systems.Drive;
import org.usfirst.frc.team4488.robot.systems.LEDController;
import org.usfirst.frc.team4488.robot.systems.SubsystemManager;
import org.usfirst.frc.team4488.robot.systems.Turret;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the IterativeRobot documentation. If you change the name of this class
 * or the package after creating this project, you must also update the manifest file in the
 * resource directory.
 */
public class Robot extends TimedRobot {

  private Drive drive;
  private Looper looper;
  private Looper constantLooper;

  private AutoModeExecuter mAutoModeExecuter = null;
  private RobotState robotState = RobotState.getInstance();
  private Logging logger = Logging.getInstance();
  private Climber climber = Climber.getInstance();
  private Arm arm = Arm.getInstance();
  public static Timer timer;
  private SubsystemManager subsystemManager;
  public static boolean isAuto = false;

  /** Run once as soon as code is loaded onto the RoboRio */
  @Override
  public void robotInit() {
    logger.writeRaw("Robot Started!");

    looper = new Looper();
    constantLooper = new Looper();

    subsystemManager = SubsystemManager.getInstance();

    if (RobotMap.driveExists) {
      drive = Drive.getInstance();
      drive.resetAngle();
    }

    AutoModeSelector.init();

    looper.register(RobotStateEstimator.getInstance());
    subsystemManager.registerEnabledLoops(looper);
    subsystemManager.zeroSensors();
    // Register anything for the constant looper here
    constantLooper.register(Camera.getInstance());
    LEDController.getInstance().registerEnabledLoops(constantLooper);
    constantLooper.start();

    CameraServer.getInstance().addAxisCamera("Axis Cam", "10.44.88.11");

    Camera.getInstance().ledOff();

    logger.addTrackable(() -> Arm.getInstance().getAngle(), "ArmAngle", 5);
    logger.addTrackable(() -> Arm.getInstance().getLength(), "ArmLength", 5);
    logger.addTrackable(() -> Turret.getInstance().getSpecificAngle(), "TurretAngle", 5);
    logger.addTrackable(() -> Arm.getInstance().getDesiredAngle(), "ArmDesiredAngle", 5);
    logger.addTrackable(() -> Arm.getInstance().getDesiredLength(), "ArmDesiredLength", 5);
    logger.addTrackable(() -> Turret.getInstance().getTargetAngle(), "TurretDesiredAngle", 5);
    logger.addTrackable(() -> Drive.getInstance().getFrontYaw(), "FrontYaw", 5);
    logger.addTrackable(() -> Drive.getInstance().getRioYaw(), "RioYaw", 5);
    logger.addTrackable(() -> Drive.getInstance().getAngle(), "DriveAngle", 5);
    logger.addTrackable(
        () ->
            RobotState.getInstance()
                .getLatestFieldToVehicle()
                .getValue()
                .getRotation()
                .getDegrees(),
        "PoseTheta",
        5);
    logger.addTrackable(() -> RobotState.getInstance().getLatestFieldToVehicle().getValue().getTranslation().x(), "PoseX", 5);
    logger.addTrackable(() -> RobotState.getInstance().getLatestFieldToVehicle().getValue().getTranslation().y(), "PoseY", 5);

  }

  /** This function is called periodically during all modes */
  @Override
  public void robotPeriodic() {
    subsystemManager.updateSmartDashboard();
    robotState.updateSmartDashboard();
    logger.update();

    double fused = NavX.getInstance().getAHRS().getFusedHeading();
    SmartDashboard.putNumber("fused", fused > 180 ? fused - 360 : fused);
  }

  /** Run once at the beginning of autonomous mode */
  @Override
  public void autonomousInit() {
    logger = Logging.forceInstance();

    logger.addTrackable(() -> Arm.getInstance().getAngle(), "ArmAngle", 5);
    logger.addTrackable(() -> Arm.getInstance().getLength(), "ArmLength", 5);
    logger.addTrackable(() -> Turret.getInstance().getSpecificAngle(), "TurretAngle", 5);
    logger.addTrackable(() -> Arm.getInstance().getDesiredAngle(), "ArmDesiredAngle", 5);
    logger.addTrackable(() -> Arm.getInstance().getDesiredLength(), "ArmDesiredLength", 5);
    logger.addTrackable(() -> Turret.getInstance().getTargetAngle(), "TurretDesiredAngle", 5);
    logger.addTrackable(
        () ->
            RobotState.getInstance()
                .getLatestFieldToVehicle()
                .getValue()
                .getRotation()
                .getDegrees(),
        "PoseTheta",
        5);
    logger.addTrackable(() -> Drive.getInstance().getAngle(), "DriveAngle", 5);
    logger.addTrackable(() -> Drive.getInstance().getFrontYaw(), "FrontYaw", 5);
    logger.addTrackable(() -> Drive.getInstance().getRioYaw(), "RioYaw", 5);
    logger.addTrackable(() -> RobotState.getInstance().getLatestFieldToVehicle().getValue().getTranslation().x(), "PoseX", 5);
    logger.addTrackable(() -> RobotState.getInstance().getLatestFieldToVehicle().getValue().getTranslation().y(), "PoseY", 5);

    isAuto = true;
    logger.writeRaw("Autonomous Init");
    looper.start();
    /*
     * if (mAutoModeExecuter != null) { mAutoModeExecuter.stop(); }
     */

    if (RobotMap.driveExists) {
      drive.resetAngle();
    }

    if (RobotMap.driveExists == true) {
      Drive.getInstance().configPercentVbus();
    }

    // mAutoModeExecuter = null;
    subsystemManager.zeroSensors();
    wait(250); // Encoder values and other sensor data is not valid until 250ms after they are
    // reset

    /*
     * mAutoModeExecuter = new AutoModeExecuter();
     * mAutoModeExecuter.setAutoMode(AutoModeSelector.getSelectedAutoMode());
     * mAutoModeExecuter.start();
     */
  }

  @Override
  public void teleopInit() {
    isAuto = false;
    logger.writeRaw("Teleop Init");

    if (RobotMap.driveExists == true) {
      Drive.getInstance().configPercentVbus();
    }

    looper.start();
  }

  /** This function is called periodically during autonomous */
  @Override
  public void autonomousPeriodic() {
    teleopPeriodic();
  }

  /** This function is called periodically during operator control */
  @Override
  public void teleopPeriodic() {
    Controllers xbox = Controllers.getInstance();

    if (xbox.getStart(xbox.m_secondary)) {
      subsystemManager.reset();
    }

    subsystemManager.controllerUpdates();
  }

  @Override
  public void testInit() {
    logger.writeRaw("Test Init");
  }

  /** This function is called periodically during test mode */
  @Override
  public void testPeriodic() {}

  /** This function is called once as soon as the robot is disabled */
  @Override
  public void disabledInit() {
    logger.writeRaw("Robot Disabled!");

    if (mAutoModeExecuter != null) {
      mAutoModeExecuter.stop();
    }

    mAutoModeExecuter = null;

    subsystemManager.updatePrefs();
    subsystemManager.reset();
    subsystemManager.stop();
    looper.stop();

    logger.flush();
  }

  @Override
  public void disabledPeriodic() {}

  private void wait(int milliseconds) {
    try {
      Thread.sleep(milliseconds);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }
}
