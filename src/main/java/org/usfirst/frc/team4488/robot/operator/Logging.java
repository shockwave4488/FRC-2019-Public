package org.usfirst.frc.team4488.robot.operator;

import edu.wpi.first.wpilibj.Timer;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;

public class Logging {

  public static String defaultPath = "/home/lvuser/NewLogs";
  public static Logging instance;

  public String fullPath;
  private BufferedWriter mainWriter;
  private ArrayList<Tracker> trackers = new ArrayList<Tracker>();

  public static Logging getInstance() {
    if (instance == null) {
      instance = new Logging(defaultPath);
    }

    return instance;
  }

  public static Logging getInstance(String path) {
    if (instance == null) {
      instance = new Logging(path);
    }

    return instance;
  }

  public boolean testLogging() {
    return instance != null;
  }

  public boolean testMainWriter() {
    return mainWriter != null;
  }

  public static Logging forceInstance() {
    instance = new Logging(defaultPath);
    return instance;
  }

  private Logging(String startPath) {
    String time = timeStamp();
    String date = dateStamp();
    fullPath = startPath + "/" + date + "/" + time;

    File directory = new File(fullPath);
    if (!directory.exists()) {
      directory.mkdirs();
    }

    try {
      mainWriter = new BufferedWriter(new FileWriter(fullPath + "/main.txt"));
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  public void update() {
    for (int counter = 0; counter < trackers.size(); counter++) {
      trackers.get(counter).update();
    }
  }

  public void addTrackable(Trackable target, String name, int frequency) {
    Tracker newTracker = new Tracker(target, fullPath, name, frequency);
    trackers.add(newTracker);
  }

  public void writeToLogFormatted(Object caller, String message) {
    String callerName = caller.getClass().getSimpleName();

    if (mainWriter != null) {
      try {
        String timestamp = Double.toString(Timer.getFPGATimestamp());
        mainWriter.write(timestamp + "\t" + callerName + "\t" + message + "\n");
      } catch (IOException e) {
        e.printStackTrace();
      }
    }
  }

  public void writeRaw(String message) {
    if (mainWriter != null) {
      try {
        mainWriter.write(message + "\n");
      } catch (IOException e) {
        e.printStackTrace();
      }
    }
  }

  public void flush() {
    try {
      mainWriter.flush();
      for (Tracker tracker : trackers) {
        tracker.flush();
      }
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  private String timeStamp() {
    Date date = new Date();
    SimpleDateFormat sdf = new SimpleDateFormat("HH-mm-ss-SSSS");
    String dateString = sdf.format(date);
    dateString.replaceAll(":", "-");
    return dateString;
  }

  private String dateStamp() {
    Date date = new Date();
    SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd");
    String dateString = sdf.format(date);
    dateString.replaceAll(":", "-");
    return dateString;
  }
}
