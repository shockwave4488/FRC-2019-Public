package org.usfirst.frc.team4488.robot.systems.drive;

import JavaRoboticsLib.ControlSystems.SimPID;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4488.lib.util.PreferenceDoesNotExistException;
import org.usfirst.frc.team4488.lib.util.PreferencesParser;
import org.usfirst.frc.team4488.robot.loops.Looper;
import org.usfirst.frc.team4488.robot.operator.Logging;
import org.usfirst.frc.team4488.robot.systems.Subsystem;

public class SmartDrive extends Subsystem {

  private Drive m_drive;
  private SimPID m_turnController;
  private SimPID m_driveController;
  private SimPID m_driveSpeedController;
  private SimPID m_straightController;
  private SimPID singleSideController;
  private PreferencesParser prefs;
  private AHRS m_navx;

  // private AnalogInput USRight;
  // private AnalogInput USLeft;

  private final int USLeftSampleLength = 5;
  private double[] USLeftSamples;
  private int USLeftSampleIndex;

  private final int USRightSampleLength = 5;
  private double[] USRightSamples;
  private int USRightSampleIndex;

  private boolean doneGear;

  private double m_driveTurnCrawlRangeMin;
  private double m_driveTurnCrawlPower; // Used to break static friction and
  private double rangeMax; // help PID converge on angle

  private double stationAngle;

  private Logging logger;

  public SmartDrive(Drive drive) {

    logger = Logging.getInstance();

    m_drive = drive;
    m_navx = drive.getGyroscope();
    // USRight = new AnalogInput(RobotMap.kUltraSonicRight);
    // USLeft = new AnalogInput(RobotMap.kUltraSonicLeft);
    doneGear = false;
    stationAngle = 63.3;
    try {
      prefs = PreferencesParser.getInstance();
      m_turnController =
          new SimPID(
              prefs.getDouble("DriveTurnP"),
              prefs.getDouble("DriveTurnI"),
              prefs.getDouble("DriveTurnD"),
              prefs.getDouble("DriveTurnEps"));
      m_turnController.setMaxOutput(1.0);
      m_turnController.setDoneRange(prefs.getDouble("DriveTurnDoneRange"));
      m_turnController.setMinDoneCycles(1);
      m_driveTurnCrawlRangeMin = 99;
      rangeMax = 101;
      try {
        m_driveController =
            new SimPID(
                prefs.getDouble("DriveP"),
                prefs.getDouble("DriveI"),
                prefs.getDouble("DriveD"),
                prefs.getDouble("DriveEps"));
      } catch (PreferenceDoesNotExistException e) {
        m_driveController = new SimPID(0.01, 0, 0, 1);
      }
      m_driveController.setMaxOutput(prefs.getDouble("DriveMaxOutput"));
      m_driveController.setDoneRange(0.5);
      m_driveController.setMinDoneCycles(5);
      m_driveSpeedController =
          new SimPID(
              prefs.getDouble("DriveSpeedP"),
              prefs.getDouble("DriveSpeedI"),
              prefs.getDouble("DriveSpeedD"),
              prefs.getDouble("DriveSpeedEps"));
      m_driveSpeedController.setMaxOutput(1);
      m_straightController =
          new SimPID(
              prefs.getDouble("DriveStraightP"),
              prefs.getDouble("DriveStraightI"),
              prefs.getDouble("DriveStraightD"),
              prefs.getDouble("DriveStraightEps"));
      m_straightController.setMaxOutput(0.20);
      m_driveTurnCrawlPower = .2;
      singleSideController = new SimPID(.11, .01, .1, 0);
      singleSideController.setMaxOutput(.67);
      singleSideController.setDoneRange(1.5);
      singleSideController.setMinDoneCycles(5);

      USLeftSamples = new double[USLeftSampleLength];
      for (int i = 0; i < USLeftSampleLength; i++) {
        USLeftSamples[i] = 0;
      }
      USLeftSampleIndex = 0;

      USRightSamples = new double[USRightSampleLength];
      for (int i = 0; i < USRightSampleLength; i++) {
        USRightSamples[i] = 0;
      }
      USRightSampleIndex = 0;
    } catch (Exception e) {
      System.out.println("Oops");
      e.printStackTrace();
    }
  }

  /** @return average of filtered ultra sonics */
  public double getUSLeftFiltered() {
    // USLeftSamples[USLeftSampleIndex] = getUSLeftDistance();
    USLeftSampleIndex = (USLeftSampleIndex + 1) % USLeftSampleLength;
    double m_tempSensorSum = 0;
    for (int i = 0; i < USLeftSampleLength; i++) {
      m_tempSensorSum += USLeftSamples[i];
    }

    return (m_tempSensorSum / USLeftSampleLength) + 1.75;
  }

  /** @return average of filtered ultra sonics */
  public double getUSRightFiltered() {
    // USRightSamples[USRightSampleIndex] = getUSRightDistance();
    USRightSampleIndex = (USRightSampleIndex + 1) % USRightSampleLength;
    double m_tempSensorSum = 0;
    for (int i = 0; i < USRightSampleLength; i++) {
      m_tempSensorSum += USRightSamples[i];
    }

    return (m_tempSensorSum / USRightSampleLength) + 1.75;
  }

  public void setLinearDoneRange(double doneRange) {
    m_driveController.setDoneRange(doneRange);
  }

  /*private double getUSLeftDistance() {
    return (USLeft.getVoltage() * 1000.0) / 9.8;
  }

  private double getUSRightDistance() {
    return (USRight.getVoltage() * 1000.0) / 9.8;
  }

  public double getUSDistAverage() {
    return (getUSLeftFiltered() + getUSRightFiltered()) / 2;
  }

  public double getUSLeftVoltage() {
    return USLeft.getVoltage();
  }

  public double getUSRightVoltage() {
    return USRight.getVoltage();
  }*/
  /** @return drive */
  public Drive getDrive() {
    return m_drive;
  }

  public void centerToWall(double side, boolean hopper) {
    m_turnController.setDesiredValue(BoundAngleNeg180To180Degrees(stationAngle * side));
    double power = m_turnController.calcPID(m_drive.getAngle());
    if (m_turnController.isDone()) power = 0;
    m_drive.setPowers(power, -power);
  }

  public void turnToCameraCrawl(boolean shooting) {
    turnToCameraCrawl(0, shooting);
  }

  public void turnToCameraCrawl(double linearPower, boolean shooting) {
    double azimuthX =
        BoundAngleNeg180To180Degrees(
            m_drive.getAngle() - SmartDashboard.getNumber("BoilerDegToCenterOfTarget", 0) - 1.5);

    m_turnController.setDesiredValue(azimuthX);

    double power = m_turnController.calcPID(m_drive.getAngle());

    if (m_turnController.isDone()) {
      power = 0;
    }

    if (Math.abs(linearPower) < m_driveTurnCrawlPower && !m_turnController.isDone() && !shooting) {
      double range = SmartDashboard.getNumber("BoilerRange", 0);
      m_driveController.setDesiredValue(90.0);
      if (range < 84.0 || range > 96) {
        linearPower = m_driveController.calcPID(range) * .6;
      } else if (range < 88 || range > 92) {
        linearPower = m_driveController.calcPID(range) * .05;
      } else linearPower = 0;
    } else linearPower = 0;

    m_drive.setPowers(linearPower + (power * .8), linearPower + (-power * .8));
  }

  public void turnToCamera() {
    double azimuthX =
        BoundAngleNeg180To180Degrees(
            m_drive.getAngle() - SmartDashboard.getNumber("BoilerDegToCenterOfTarget", 0) - 1.5);

    m_turnController.setDesiredValue(azimuthX);

    double power = m_turnController.calcPID(m_drive.getAngle());

    if (m_turnController.isDone()) {
      power = 0;
    }

    m_drive.setPowers((power * .8), -power * .8);
  }
  /** @return the angle needed to turn to get to the desired angle */
  public double getAzimuthX() {
    return BoundAngleNeg180To180Degrees(
        m_drive.getAngle() - SmartDashboard.getNumber("BoilerDegToCenterOfTarget", 0));
  }

  /** @return the desired PID value of where to drive */
  public double getTurnSetpoint() {
    return m_turnController.getDesiredVal();
  }

  public void arcDrive(double heading, boolean reverse) {
    double power = 0.0;
    m_straightController.resetErrorSum();
    m_straightController.resetPreviousVal();

    m_straightController.setDesiredValue(BoundAngleNeg180To180Degrees(heading));
    double angleCorrection = m_straightController.calcPID(m_drive.getAngle());
    if (reverse) {
      power = -0.3;
    } else {
      power = 0.3;
    }
    final double angleCorrectionMultiplier = 3;
    angleCorrection = angleCorrection * angleCorrectionMultiplier;
    if ((angleCorrection + power) > 1) {
      m_drive.setPowers(1 / 1.5, (power - angleCorrection) / 1.5);
    } else if ((angleCorrection - power) < -1) {
      m_drive.setPowers((power + angleCorrection) / 1.5, -1 / 1.5);
    } else {
      m_drive.setPowers((power + angleCorrection) / 1.5, (power - angleCorrection) / 1.5);
    }
  }

  public void driveToDistance(double distance) {
    m_driveController.setDesiredValue(distance);
    double power = m_driveController.calcPID(m_drive.getLinearDistance());
    m_drive.setPowers(power, power);
  }

  /** @return the desired distance to travel */
  public double getDesiredDriveDistance() {
    return m_driveController.getDesiredVal();
  }

  public void driveToDistance(double distance, double heading) {
    m_driveController.setDesiredValue(distance);
    m_straightController.setDesiredValue(heading);
    double power = m_driveController.calcPID(m_drive.getLinearDistance());
    double angleCorrection = m_straightController.calcPID(m_drive.getAngle());
    m_drive.setPowers(power + angleCorrection, power - angleCorrection);
  }

  public void driveToSpeed(double speed, double heading) {
    m_driveSpeedController.setDesiredValue(speed);
    m_straightController.setDesiredValue(heading);
    double power = m_driveSpeedController.calcPID(m_drive.getLinearSpeed());
    double angleCorrection = m_straightController.calcPID(m_drive.getAngle());
    m_drive.setPowers(power + angleCorrection, power - angleCorrection);
  }

  public void turnToAngle(double angle) {
    m_turnController.setDesiredValue(angle);
    double power = m_turnController.calcPID(m_drive.getAngle());
    m_drive.setPowers((power), (-power));
  }

  public void turnToAngleLeftSide(double angle) {
    singleSideController.setDesiredValue(angle);
    double power = singleSideController.calcPID(m_drive.getAngle());
    m_drive.setPowers((power + m_driveTurnCrawlPower), 0);
  }

  public void turnToAngleRightSide(double angle) {
    singleSideController.setDesiredValue(angle);
    double power = singleSideController.calcPID(m_drive.getAngle());
    m_drive.setPowers(0, (-power - m_driveTurnCrawlPower));
  }

  public void stop() {
    m_drive.setPowers(0, 0);
  }

  public void resetAll() {
    m_drive.resetAngle();
    m_drive.resetEncoders();
  }

  /** @return true when target is found */
  public boolean TargetFound() {
    if (Math.abs(SmartDashboard.getNumber("BoilerDegToCenterOfTarget", 0)) < 29) {
      return true;
    } else return false;
  }

  /** @return true if drive distance is done */
  public boolean isDriveDistanceDone() {
    return m_driveController.isDone();
  }
  /**
   * @param heading
   * @return true if drive turn is done
   */
  public boolean isDriveTurnDone(double heading) {
    if (heading > 0) {
      if (m_navx.getYaw() >= heading) {
        return true;
      } else {
        return false;
      }
    } else {
      if (m_navx.getYaw() <= heading) {
        return true;
      } else {
        return false;
      }
    }
  }
  /** @return true if turn is done */
  public boolean isTurnDone() {
    return m_turnController.isDone();
  }
  /** @return true if single side turn is done */
  public boolean isSingleSideTurnDone() {
    return singleSideController.isDone();
  }

  public void setDriveMaxOutput(double max) {
    m_driveController.setMaxOutput(max);
  }
  /** @return Drive max output */
  public double getDriveMaxOutput() {
    return m_driveController.getMaxOutputVal();
  }

  public void setTurnDoneRange(double range) {
    m_turnController.setDoneRange(range);
  }
  /** @return Turn done range */
  public double getTurnDoneRange() {
    return m_turnController.getDoneRangeVal();
  }

  public void setDriveDoneRange(double range) {
    m_driveController.setDoneRange(range);
  }
  /** @return Drive done range */
  public double getDriveDoneRange() {
    return m_driveController.getDoneRangeVal();
  }

  public void setTurnMinDoneCycles(int cycles) {
    m_turnController.setMinDoneCycles(cycles);
  }

  /** @return Minimum turn for done cycle */
  public int getTurnMinDoneCycles() {
    return m_turnController.getMinDoneCycles();
  }
  /** @return Turn desired value. */
  public double getTurnDesValue() {
    return m_turnController.getDesiredVal();
  }

  public void setDriveMinDoneCycles(int cycles) {
    m_driveController.setMinDoneCycles(cycles);
  }
  /** @return minimum cycles done for drive */
  public int getDriveMinDoneCycles() {
    return m_driveController.getMinDoneCycles();
  }

  /**
   * @param angle
   * @return turns positive angles to negative and negative to positive
   */
  private double BoundAngleNeg180To180Degrees(double angle) {
    while (angle <= -180) angle += 360;
    while (angle > 180) angle -= 360;
    return angle;
  }

  /** @return true if you are close enough to the boiler to score */
  public boolean inShooterRange() {
    if (SmartDashboard.getNumber("BoilerRange", 0) >= 80
        && SmartDashboard.getNumber("BoilerRange", 0) <= 120) {
      return true;
    } else return false;
  }

  private void wait(int milliseconds) {
    try {
      Thread.sleep(milliseconds);
    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }

  @Override
  public void updateSmartDashboard() {}

  @Override
  public void zeroSensors() {
    resetAll();
  }

  @Override
  public void registerEnabledLoops(Looper enabledLooper) {}

  public void updatePrefs() {
    PreferencesParser prefs = PreferencesParser.getInstance();
    try {
      m_driveController =
          new SimPID(
              prefs.getDouble("DriveP"),
              prefs.getDouble("DriveI"),
              prefs.getDouble("DriveD"),
              prefs.getDouble("DriveEps"));
      m_turnController =
          new SimPID(
              prefs.getDouble("DriveTurnP"),
              prefs.getDouble("DriveTurnI"),
              prefs.getDouble("DriveTurnD"),
              prefs.getDouble("DriveTurnEps"));
      m_turnController.setMaxOutput(1.0);
      m_turnController.setDoneRange(prefs.getDouble("DriveTurnDoneRange"));
      m_turnController.setMinDoneCycles(1);
      m_straightController =
          new SimPID(
              prefs.getDouble("DriveStraightP"),
              prefs.getDouble("DriveStraightI"),
              prefs.getDouble("DriveStraightD"),
              prefs.getDouble("DriveStraightEps"));

    } catch (PreferenceDoesNotExistException e) {
      m_driveController = new SimPID(0.01, 0, 0, 1);
    }
  }

  @Override
  public void writeToLog() {}

  @Override
  public void reset() {}
}
