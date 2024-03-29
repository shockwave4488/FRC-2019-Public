package org.usfirst.frc.team4488.robot.sensors;

public class Potentiometer extends Sensor {

  public Potentiometer(int port) {
    super(port, Type.Analog);
  }

  public double get() {
    return ((Integer) getObjectValue()).doubleValue();
  }

  @Override
  public void reset() {}

  @Override
  public void loop() {}
}
