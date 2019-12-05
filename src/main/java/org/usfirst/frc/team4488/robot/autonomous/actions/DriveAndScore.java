package org.usfirst.frc.team4488.robot.autonomous.actions;

import java.util.ArrayList;

public class DriveAndScore implements Action {

  private SeriesAction routine;

  public DriveAndScore() {
    ArrayList<Action> mode = new ArrayList<Action>();
    mode.add(new GenerateAndDrivePathAction(76, 76));
    mode.add(new TurnToHeadingAction(-150));
    mode.add(new DirectDriveStraightAction(12));
    routine = new SeriesAction(mode);
  }

  @Override
  public void start() {
    routine.start();
  }

  @Override
  public void update() {
    routine.update();
  }

  @Override
  public void done() {
    routine.done();
  }

  @Override
  public boolean isFinished() {
    return routine.isFinished();
  }
}
