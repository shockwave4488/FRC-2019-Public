package org.usfirst.frc.team4488.robot.sensors;

import edu.wpi.first.wpilibj.AnalogInput;

public class ShockSonic {
  private double currentDistance;
  private AnalogInput US0;
  private double VOLTS_TO_MILLIVOLTS = 1000.0;
  private double MILLIVOLTS_PER_INCH = 9.8;

  public ShockSonic(int channel) {
    US0 = new AnalogInput(channel);
  }

  public double getDistance() {
    currentDistance = (US0.getVoltage() * VOLTS_TO_MILLIVOLTS) / MILLIVOLTS_PER_INCH;
    // Numbers that were used last year to convert getVoltage and etc. to inches.
    // 9.8mV per inch
    return currentDistance;
  }

  public boolean withinProximity(double distance) {
    if (getDistance() < distance) {
      return true;
    }
    return false;
  }
}
