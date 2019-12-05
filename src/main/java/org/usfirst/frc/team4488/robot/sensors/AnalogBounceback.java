package org.usfirst.frc.team4488.robot.sensors;

import JavaRoboticsLib.ControlSystems.SetPointProfile;

public class AnalogBounceback extends Sensor {

  private SetPointProfile interpolTable;
  private double cmToInches = 2.54;

  public static final double MINIMUM = 20;
  public static final double MAXIMUM = 160;

  public AnalogBounceback(int channel) {
    super(channel, Type.Analog);

    interpolTable = new SetPointProfile();
    interpolTable.add(2.36, MINIMUM);
    interpolTable.add(2.13, 25);
    interpolTable.add(1.86, 30);
    interpolTable.add(1.65, 35);
    interpolTable.add(1.46, 40);
    interpolTable.add(1.22, 50);
    interpolTable.add(1.08, 60);
    interpolTable.add(0.98, 70);
    interpolTable.add(0.91, 80);
    interpolTable.add(0.77, 120);
    interpolTable.add(0.68, MAXIMUM);
  }

  public int getRaw() {
    return (int) getObjectValue();
  }

  public double get() {
    double data = interpolTable.get(getRaw()); // Returns centimeters
    return isValidData(data) ? data / cmToInches : 0;
  }

  public boolean isValidData(double data) {
    return (data <= MINIMUM || data >= MAXIMUM) ? false : true;
  }

  @Override
  public void reset() {}

  @Override
  public void loop() {}
}
