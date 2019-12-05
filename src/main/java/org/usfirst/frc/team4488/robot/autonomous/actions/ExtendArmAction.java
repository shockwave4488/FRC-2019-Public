package org.usfirst.frc.team4488.robot.autonomous.actions;

import org.usfirst.frc.team4488.robot.systems.Arm;
import org.usfirst.frc.team4488.robot.systems.SubsystemManager;

public class ExtendArmAction implements Action {

  private Arm arm = Arm.getInstance();
  private SubsystemManager manager = SubsystemManager.getInstance();
  private double ext;
  private double doneRange;

  public ExtendArmAction(double ext, double doneRange) {
    this.ext = ext;
    this.doneRange = doneRange;
  }

  @Override
  public void start() {
    manager.safeExtendArm(ext);
  }

  @Override
  public void update() {
    manager.safeExtendArm(ext);
  }

  @Override
  public void done() {}

  @Override
  public boolean isFinished() {
    return Math.abs(arm.getLength() - ext) < doneRange;
  }
}
