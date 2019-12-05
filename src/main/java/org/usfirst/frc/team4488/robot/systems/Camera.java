package org.usfirst.frc.team4488.robot.systems;

import JavaRoboticsLib.ControlSystems.SetPointProfile;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4488.lib.util.PreferencesParser;
import org.usfirst.frc.team4488.robot.RobotMap;
import org.usfirst.frc.team4488.robot.loops.Loop;

public class Camera implements Loop {

  public static class CameraFrame {
    public double x;
    public double y;
    public double dist;
    public LastTiltRead tilt;

    public CameraFrame() {
      x = Double.NaN;
      y = Double.NaN;
      dist = Double.NaN;
      tilt = LastTiltRead.None;
    }
  }

  private CameraFrame[] reads =
      new CameraFrame[] {new CameraFrame(), new CameraFrame(), new CameraFrame()};
  private CameraFrame[] readsBottom =
      new CameraFrame[] {new CameraFrame(), new CameraFrame(), new CameraFrame()};
  private SerialPort serial;
  private SerialPort serialBottom;
  private DigitalOutput led;
  public double cameraWidthPixels = 320;
  public double cameraFOV = 65;
  public double cameraFOVBottom = 90;
  private double offset = 15;

  private SetPointProfile areaToDist = new SetPointProfile();

  public enum LastTiltRead {
    Left(0),
    Right(1),
    Both(2),
    None(3);

    private int val;

    private LastTiltRead(int val) {
      this.val = val;
    }
  }

  private static Camera sInstance;

  public static Camera getInstance() {
    if (sInstance == null) {
      sInstance = new Camera();
    }

    return sInstance;
  }

  public Camera() {
    init();
    led = new DigitalOutput(RobotMap.TopJevoisLED);

    areaToDist.add(3550, 17);
    areaToDist.add(2920, 20.5);
    areaToDist.add(2120, 24.5);
    areaToDist.add(1610, 27);
    areaToDist.add(1292, 30.25);
    areaToDist.add(1108, 32.5);
    areaToDist.add(976, 35);
    areaToDist.add(770, 39);
    areaToDist.add(660, 42);
    areaToDist.add(515, 47.5);

    offset = PreferencesParser.getInstance().getDouble("CamOffset");
  }

  private void init() {
    try {
      serial = new SerialPort(115200, Port.kUSB);
    } catch (Exception e) {
      System.out.println("Could not instantiate camera serial port");
    }
  }

  private void initBottom() {
    try {
      serialBottom = new SerialPort(115200, Port.kMXP);
    } catch (Exception e) {
      System.out.println("Could not instantiate camera serialBottom port");
    }
  }

  public CameraFrame getLastReading() {
    return reads[0];
  }

  public CameraFrame getLastReadingBottom() {
    return readsBottom[0];
  }

  @Override
  public void onStart(double timestamp) {}

  @Override
  public void onLoop(double timestamp) {
    try {
      Thread.sleep(50);
    } catch (InterruptedException e) {
    }
    if (serial == null) {
      init();
      return;
    }
    /*
    if (serialBottom == null) {
      initBottom();
      return;
    }
    */

    String raw1 = serial.readString(50);
    if (raw1.length() == 0) {
      init();
    } else {
      addFrame(buildFrame(raw1));
    }

    /*
    String raw1Bottom = serialBottom.readString(50);
    if (raw1Bottom.length() == 0) {
      initBottom();
    } else {
      addFrameBottom(buildFrame(raw1Bottom));
    }
    */

    // SmartDashboard.putNumber("tilt", reads[0].tilt.val);
    SmartDashboard.putNumber("x", reads[0].x);
    // SmartDashboard.putNumber("y", reads[0].y);
    // SmartDashboard.putNumber("size", reads[0].dist);

    // SmartDashboard.putNumber("tiltBottom", readsBottom[0].tilt.val);
    // SmartDashboard.putNumber("xBottom", readsBottom[0].x);
    // SmartDashboard.putNumber("yBottom", readsBottom[0].y);
  }

  private CameraFrame buildFrame(String read) {
    CameraFrame frame = new CameraFrame();
    try {
      String message = read.split("&")[1];
      if (message.equals("None")) {
        frame.x = Double.NaN;
        frame.y = Double.NaN;
        frame.tilt = LastTiltRead.None;
        return frame;
      }
      frame.x = Double.valueOf(message.split(",")[0]) - offset;
      frame.y = Double.valueOf(message.split(",")[1]);
      if (message.split(",")[2].equals("B")) {
        frame.tilt = LastTiltRead.Both;
      } else if (message.split(",")[2].equals("L")) {
        frame.tilt = LastTiltRead.Left;
      } else if (message.split(",")[2].equals("R")) {
        frame.tilt = LastTiltRead.Right;
      }
      frame.dist = areaToDist.get(Double.valueOf(message.split(",")[3]));

    } catch (ArrayIndexOutOfBoundsException e) {
      System.out.println("Camera came unplugged / sent bad data");
      frame.x = Double.NaN;
      frame.y = Double.NaN;
      frame.dist = Double.NaN;
      frame.tilt = LastTiltRead.None;
    } catch (NumberFormatException e) {
      // Camera either sent bad data or doesnt see target
      frame.x = Double.NaN;
      frame.y = Double.NaN;
      frame.dist = Double.NaN;
      frame.tilt = LastTiltRead.None;
    }

    return frame;
  }

  private void addFrame(CameraFrame frame) {
    reads[2] = reads[1];
    reads[1] = reads[0];
    reads[0] = frame;
  }

  private void addFrameBottom(CameraFrame frame) {
    readsBottom[2] = readsBottom[1];
    readsBottom[1] = readsBottom[0];
    readsBottom[0] = frame;
  }

  public boolean targetFound() {
    int good = 0;
    for (CameraFrame frame : reads) {
      if (frame.tilt != LastTiltRead.None) good++;
    }
    return good >= 2;
  }

  public boolean targetFoundBottom() {
    int good = 0;
    for (CameraFrame frame : readsBottom) {
      if (frame.tilt != LastTiltRead.None) good++;
    }
    return good >= 2;
  }

  public void ledOn() {
    led.set(true);
  }

  public void ledOff() {
    led.set(false);
  }

  @Override
  public void onStop(double timestamp) {
    ledOff();
  }
}
