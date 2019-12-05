package org.usfirst.frc.team4488.robot.autonomous.actions;

public class Wait extends RunOnceAction {

  private int millis;

  public Wait(int millis) {
    this.millis = millis;
  }

  @Override
  public void runOnce() {
    try {
      Thread.sleep(millis);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }
}
