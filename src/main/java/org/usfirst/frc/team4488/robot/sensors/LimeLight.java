package org.usfirst.frc.team4488.robot.sensors;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLight {

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  private static LimeLight sInstance;

  public static LimeLight getInstance() {
    if (sInstance == null) {
      sInstance = new LimeLight();
    }
    return sInstance;
  }

  /*
   * returns if limelight has target
   */
  public boolean hasTarget() {
    return table.getEntry("tv").getDouble(0) == 1;
  }

  /*
   *returns x value of target from crosshair
   */
  public double getX() {
    return table.getEntry("tx").getDouble(0);
  }

  /*
   *returns y value of target from crosshair
   */
  public double getY() {
    return table.getEntry("ty").getDouble(0);
  }

  /*
   *turns limelight led on
   */
  public void ledOn() {
    table.getEntry("ledMode").setNumber(3);
  }

  /*
   *turns limelight led off
   */
  public void ledOff() {
    table.getEntry("ledMode").setNumber(1);
  }

  /*
   *limelight led starts
   */
  public void ledBlink() {
    table.getEntry("ledMode").setNumber(2);
  }

  /*
   *limelight led uses default pipeline settings
   */
  public void ledDefault() {
    table.getEntry("ledMode").setNumber(0);
  }

  /*
   *limelight takes a picture!, limited to 2 per second
   *saved on limelight, source image, snapshot
   */
  public void takePicture() {
    table.getEntry("snapshot").setNumber(1);
  }
}
