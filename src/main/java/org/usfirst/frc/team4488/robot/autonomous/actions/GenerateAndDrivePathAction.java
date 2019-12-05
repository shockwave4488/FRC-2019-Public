package org.usfirst.frc.team4488.robot.autonomous.actions;

import org.usfirst.frc.team4488.lib.util.app.math.Translation2d;
import org.usfirst.frc.team4488.robot.autonomous.PathFinder;
import org.usfirst.frc.team4488.robot.loops.RobotStateLoop;

public class GenerateAndDrivePathAction implements Action {

  private double goalX;
  private double goalY;

  private DrivePathAction driveAction;

  public GenerateAndDrivePathAction(double goalX, double goalY) {
    this.goalX = goalX;
    this.goalY = goalY;
  }

  @Override
  public void start() {
    Translation2d pos =
        RobotStateLoop.getInstance()
            .getEstimator()
            .getLatestFieldToVehicle()
            .getValue()
            .getTranslation();
    driveAction =
        new DrivePathAction(
            PathFinder.generatePath((int) pos.x(), (int) pos.y(), (int) goalX, (int) goalY));
    driveAction.start();
  }

  @Override
  public void update() {
    driveAction.update();
  }

  @Override
  public void done() {
    driveAction.done();
  }

  @Override
  public boolean isFinished() {
    return driveAction.isFinished();
  }
}
