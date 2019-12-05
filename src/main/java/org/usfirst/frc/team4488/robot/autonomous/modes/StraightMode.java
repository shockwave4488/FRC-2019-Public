package org.usfirst.frc.team4488.robot.autonomous.modes;

import org.usfirst.frc.team4488.robot.app.paths.Straight;
import org.usfirst.frc.team4488.robot.autonomous.AutoModeBase;
import org.usfirst.frc.team4488.robot.autonomous.AutoModeEndedException;
import org.usfirst.frc.team4488.robot.autonomous.actions.DrivePathAction;

public class StraightMode extends AutoModeBase {

  @Override
  protected void routine() throws AutoModeEndedException {
    DrivePathAction action = new DrivePathAction(new Straight());
    runAction(action);
  }
}
