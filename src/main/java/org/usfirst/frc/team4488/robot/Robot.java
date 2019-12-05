package org.usfirst.frc.team4488.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team4488.lib.util.NavX;
import org.usfirst.frc.team4488.robot.autonomous.AutoModeExecuter;
import org.usfirst.frc.team4488.robot.autonomous.AutoModeSelector;
import org.usfirst.frc.team4488.robot.loops.Looper;
import org.usfirst.frc.team4488.robot.loops.RobotStateLoop;
import org.usfirst.frc.team4488.robot.operator.Controllers;
import org.usfirst.frc.team4488.robot.operator.Logging;
import org.usfirst.frc.team4488.robot.systems.SpiderBotClimb;
import org.usfirst.frc.team4488.robot.systems.SubsystemManager;
import org.usfirst.frc.team4488.robot.systems.drive.DriveBase;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the IterativeRobot documentation. If you change the name of this class
 * or the package after creating this project, you must also update the manifest file in the
 * resource directory.
 */
public class Robot extends TimedRobot {

  private Looper looper;
  private Looper constantLooper;

  private AutoModeExecuter mAutoModeExecuter = null;
  private Logging logger = Logging.getInstance();
  public static Timer timer;
  private SubsystemManager subsystemManager;
  public static boolean isAuto = false;
  private DriveBase drive;

  /** Run once as soon as code is loaded onto the RoboRio */
  @Override
  public void robotInit() {
    logger.writeToLogFormatted(this, "Robot Started!");

    looper = new Looper();
    constantLooper = new Looper();

    RobotSystems systems = RobotMap.robotSelector();
    drive = systems.drive;
    subsystemManager = SubsystemManager.createInstance(systems);
    looper.register(RobotStateLoop.createInstance(drive.getStateEstimator()));

    AutoModeSelector.init();

    subsystemManager.registerEnabledLoops(looper);
    subsystemManager.zeroSensors();

    // addTrackables(logger);

    logger.addStringTrackable(
        () -> Controllers.getInstance().getPrimaryControllerLogging(),
        "PrimaryContLog",
        20,
        "TimeStamp,A,B,X,Y,LBump,RBump,LTrig,RTrig,LStickPress,RStickPress,Start,Select,LStickX,LStickY,RStickX,RStickY,Dpad\n");
    logger.addStringTrackable(
        () -> Controllers.getInstance().getSecondaryControllerLogging(),
        "SecondaryContLog",
        20,
        "TimeStamp,A,B,X,Y,LBump,RBump,LTrig,RTrig,LStickPress,RStickPress,Start,Select,LStickX,LStickY,RStickX,RStickY,Dpad\n");
    /*logger.addTrackable(
    () ->
    RobotStateLoop.getInstance().getEstimator()
            .getLatestFieldToVehicle()
            .getValue()
            .getRotation()
            .getDegrees(),
    "PoseTheta",
    5);*/

    // logger.addTrackable(() ->
    // RobotStateLoop.getInstance().getEstimator().getLatestFieldToVehicle().getValue().getTranslation().x(), "PoseX", 5);
    // logger.addTrackable(() ->
    // RobotStateLoop.getInstance().getEstimator().getLatestFieldToVehicle().getValue().getTranslation().y(), "PoseY", 5);
    // Register anything for the constant looper here
    constantLooper.start();

    // CameraServer.getInstance().addAxisCamera("Axis Cam", "10.44.88.11");
  }

  /** This function is called periodically during all modes */
  @Override
  public void robotPeriodic() {
    subsystemManager.updateSmartDashboard();
    // robotState.updateSmartDashboard();
    logger.update();
  }

  /** Run once at the beginning of autonomous mode */
  @Override
  public void autonomousInit() {
    logger = Logging.forceInstance();
    /*logger.addTrackable(
    () ->
    RobotStateLoop.getInstance().getEstimator()
            .getLatestFieldToVehicle()
            .getValue()
            .getRotation()
            .getDegrees(),
    "PoseTheta",
    5);*/
    logger.addTrackable(() -> NavX.getInstance().getYaw().getDegrees(), "DriveAngle", 5);
    // logger.addTrackable(() ->
    // RobotStateLoop.getInstance().getEstimator().getLatestFieldToVehicle().getValue().getTranslation().x(), "PoseX", 5);
    // logger.addTrackable(() ->
    // RobotStateLoop.getInstance().getEstimator().getLatestFieldToVehicle().getValue().getTranslation().y(), "PoseY", 5);

    isAuto = true;
    logger.writeToLogFormatted(this, "Autonomous Init");
    looper.start();
    /*
     * if (mAutoModeExecuter != null) { mAutoModeExecuter.stop(); }
     */

    drive.resetAngle();
    drive.configPercentVbus();

    // mAutoModeExecuter = null;
    subsystemManager.zeroSensors();
    wait(250); // Encoder values and other sensor data is not valid until 250ms after they are
    // reset
    SpiderBotClimb.getInstance().spiderUp();
    /*
     * mAutoModeExecuter = new AutoModeExecuter();
     * mAutoModeExecuter.setAutoMode(AutoModeSelector.getSelectedAutoMode());
     * mAutoModeExecuter.start();
     */

    // addTrackables(logger);
  }

  @Override
  public void teleopInit() {
    isAuto = false;
    logger.writeToLogFormatted(this, "Teleop Init");

    drive.configPercentVbus();

    looper.start();

    NavX.getInstance().reset();
  }

  /** This function is called periodically during autonomous */
  @Override
  public void autonomousPeriodic() {}
  /** This function is called periodically during operator control */
  @Override
  public void teleopPeriodic() {
    Controllers xbox = Controllers.getInstance();

    if (xbox.getStart(xbox.m_primary)) {
      subsystemManager.reset();
    }

    subsystemManager.controllerUpdates();
  }

  @Override
  public void testInit() {
    logger.writeToLogFormatted(this, "Test Init");
  }

  /** This function is called periodically during test mode */
  @Override
  public void testPeriodic() {}

  /** This function is called once as soon as the robot is disabled */
  @Override
  public void disabledInit() {
    logger.writeToLogFormatted(this, "Robot Disabled!");

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

  public void addTrackables(Logging logger) {
    /*logger.addTrackable(
        () ->
            RobotStateLoop.getInstance()
                .getEstimator()
                .getLatestFieldToVehicle()
                .getValue()
                .getRotation()
                .getDegrees(),
        "PoseTheta",
        5);
    logger.addTrackable(
        () ->
            RobotStateLoop.getInstance()
                .getEstimator()
                .getLatestFieldToVehicle()
                .getValue()
                .getTranslation()
                .x(),
        "PoseX",
        5);
    logger.addTrackable(
        () ->
            RobotStateLoop.getInstance()
                .getEstimator()
                .getLatestFieldToVehicle()
                .getValue()
                .getTranslation()
                .y(),
        "PoseY",
        5);*/
  }

  private void wait(int milliseconds) {
    try {
      Thread.sleep(milliseconds);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }
}
