package org.usfirst.frc.team4488.robot.autonomous.actions;

import org.usfirst.frc.team4488.robot.systems.Drive;
import org.usfirst.frc.team4488.robot.systems.SubsystemManager;
import org.usfirst.frc.team4488.robot.systems.Turret;

public class TurnTurretToFieldAction implements Action {

  private Turret turret = Turret.getInstance();
  private SubsystemManager manager = SubsystemManager.getInstance();
  private double angle;
  private double doneRange;

  public TurnTurretToFieldAction(double angle, double doneRange) {
    this.angle = angle;
    this.doneRange = doneRange;
  }

  @Override
  public void start() {
    manager.turnTurretToField(angle);
  }

  @Override
  public void update() {
    manager.turnTurretToField(angle);
  }

  @Override
  public void done() {
    manager.resetFieldTurretAngle();
  }

  @Override
  public boolean isFinished() {
    double fieldAngle = turret.getSpecificAngle() + Drive.getInstance().getAngle();
    fieldAngle += (fieldAngle < 0) ? 360 : 0;
    return Math.abs(fieldAngle - angle) < doneRange;
  }
}
