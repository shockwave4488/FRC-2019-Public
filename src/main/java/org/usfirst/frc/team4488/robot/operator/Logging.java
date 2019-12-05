package org.usfirst.frc.team4488.robot.operator;

import edu.wpi.first.wpilibj.DriverStation;
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
  private ArrayList<StringTracker> stringTrackers = new ArrayList<StringTracker>();
  private FileWriter runOnce;

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
    String date = dateStamp();

    String matchType = DriverStation.getInstance().getMatchType().name() + "_";
    String matchNumber = DriverStation.getInstance().getMatchNumber() + "_";

    if (matchType.equals("None")) {
      matchType = "";
      matchNumber = "";
    }

    fullPath = startPath + "/" + date + "/" + (matchType + matchNumber) + timeStamp();

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
    for (int counter = 0; counter < stringTrackers.size(); counter++) {
      stringTrackers.get(counter).update();
    }
  }

  public void addTrackable(Trackable target, String name, int frequency) {
    Tracker newTracker = new Tracker(target, fullPath, name, frequency);
    trackers.add(newTracker);
  }

  public void addStringTrackable(
      StringTrackable target, String name, int frequency, String header) {
    StringTracker newStringTracker = new StringTracker(target, fullPath, name, frequency, header);
    stringTrackers.add(newStringTracker);
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

  /**
   * Formatted version of logs that adheres to strategy programming standards. For use in mock build
   * week.
   *
   * @param callClass - the top level class that this function is being called from
   * @param routine - the current routine the robot is executing
   * @param system - the specific subsystem that is involved
   * @param message - the state
   */
  public void writeToLogFormatted(Object callClass, Object routine, Object system, Object message) {
    // Stores important information from object parameters
    if (callClass == null) {
      callClass = "null";
    } else if (!(callClass instanceof String)) {
      callClass = callClass.getClass().getSimpleName();
    }
    if (routine == null) {
      routine = "null";
    } else if (!(routine instanceof String)) {
      routine = routine.getClass().getSimpleName();
    }
    if (system == null) {
      system = "null";
    } else if (!(system instanceof String)) {
      system = system.getClass().getSimpleName();
    }
    if (message == null) {
      message = "null";
    } else if (!(system instanceof String)) {
      message = message.toString();
    }

    String timestamp = Double.toString(Timer.getFPGATimestamp());

    if (mainWriter != null) {
      try {
        mainWriter.write(
            timestamp + "\t" + callClass + "\t" + routine + "\t" + system + "\t" + message + "\n");
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
      for (StringTracker stringTracker : stringTrackers) {
        stringTracker.flush();
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
