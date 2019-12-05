package org.usfirst.frc.team4488.robot.autonomous.actions;

import edu.wpi.first.wpilibj.Timer;

public class TimerWait implements Action {

  private double time;
  private double startTime;

  public TimerWait(double time) {
    this.time = time;
  }

  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp() - startTime) * 1000 > time;
  }

  @Override
  public void update() {}

  @Override
  public void done() {}

  @Override
  public void start() {
    startTime = Timer.getFPGATimestamp();
  }
}
