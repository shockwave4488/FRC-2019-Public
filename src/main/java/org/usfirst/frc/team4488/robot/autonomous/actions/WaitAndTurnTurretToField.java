package org.usfirst.frc.team4488.robot.autonomous.actions;

import org.usfirst.frc.team4488.robot.systems.Drive;
import org.usfirst.frc.team4488.robot.systems.SubsystemManager;
import org.usfirst.frc.team4488.robot.systems.Turret;

import edu.wpi.first.wpilibj.Timer;

public class WaitAndTurnTurretToField implements Action {

  private Turret turret = Turret.getInstance();
  private SubsystemManager manager = SubsystemManager.getInstance();
  private double angle;
  private double doneRange;
  private double delay;
  private double startTime;

  public WaitAndTurnTurretToField(double angle, double doneRange, double delay) {
    this.angle = angle;
    this.doneRange = doneRange;
    this.delay = delay;
  }

  @Override
  public void start() {
    startTime = Timer.getFPGATimestamp() * 1000;
  }

  @Override
  public void update() {
    if((Timer.getFPGATimestamp() * 1000) - startTime > delay) {
        manager.turnTurretToField(angle);
    }
  }

  @Override
  public void done() {
    manager.resetFieldTurretAngle();
  }

  @Override
  public boolean isFinished() {
    double fieldAngle = turret.getSpecificAngle() + Drive.getInstance().getAngle();
    fieldAngle += (fieldAngle < 0) ? 360 : 0;
    return (Math.abs(fieldAngle - angle) < doneRange) && ((Timer.getFPGATimestamp() * 1000) - startTime > delay);
  }
}