package org.usfirst.frc.team4488.robot.sensors;

public class BeamBreak extends Sensor {

  public BeamBreak(int port) {
    super(port, Type.Digital);
  }

  public boolean get() {
    return (boolean) getObjectValue();
  }

  @Override
  public void reset() {}

  @Override
  public void loop() {}
}
