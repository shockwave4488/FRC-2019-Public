package org.usfirst.frc.team4488.robot.operator;

import edu.wpi.first.wpilibj.Timer;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;

public class Tracker {

  private Trackable target;
  private int frequency;
  private int lastWritten;
  private BufferedWriter file;

  public Tracker(Trackable target, String dir, String name, int frequency) {
    this.target = target;
    this.frequency = frequency;

    String path = dir + "/" + name + ".txt";

    try {
      file = new BufferedWriter(new FileWriter(path));
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  public void update() {
    if (getTimeMillis() - lastWritten > (1000 / frequency)) {
      String timestamp = Double.toString(Timer.getFPGATimestamp());
      lastWritten = getTimeMillis();

      try {
        file.write(timestamp + " " + target.get() + "\n");
      } catch (IOException e) {
        e.printStackTrace();
      }
    }
  }

  public void flush() {
    try {
      file.flush();
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  private int getTimeMillis() {
    return (int) (Timer.getFPGATimestamp() * 1000);
  }
}
