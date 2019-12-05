package org.usfirst.frc.team4488.robot.autonomous.actions;

import org.usfirst.frc.team4488.robot.systems.Arm;
import org.usfirst.frc.team4488.robot.systems.SubsystemManager;

public class RotateArmAction implements Action {

  private Arm arm = Arm.getInstance();
  private SubsystemManager manager = SubsystemManager.getInstance();
  private double angle;
  private double doneRange;

  public RotateArmAction(double angle, double doneRange) {
    this.angle = angle;
    this.doneRange = doneRange;
  }

  @Override
  public void start() {
    manager.safeAngleArm(angle);
  }

  @Override
  public void update() {
    manager.safeAngleArm(angle);
  }

  @Override
  public void done() {}

  @Override
  public boolean isFinished() {
    return Math.abs(arm.getAngle() - angle) < doneRange;
  }
}
