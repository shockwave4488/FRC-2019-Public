package org.usfirst.frc.team4488.robot.autonomous.actions;

import org.usfirst.frc.team4488.robot.systems.SubsystemManager;
import org.usfirst.frc.team4488.robot.systems.Turret;

public class TurnTurretAction implements Action {

  private Turret turret = Turret.getInstance();
  private SubsystemManager manager = SubsystemManager.getInstance();
  private double angle;
  private double doneRange;

  public TurnTurretAction(double angle, double doneRange) {
    this.angle = angle;
    this.doneRange = doneRange;
  }

  @Override
  public void start() {
    manager.safeRotateTurret(angle);
  }

  @Override
  public void update() {
    manager.safeRotateTurret(angle);
  }

  @Override
  public void done() {
    manager.resetFieldTurretAngle();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(turret.getSpecificAngle() - angle) < doneRange;
  }
}
