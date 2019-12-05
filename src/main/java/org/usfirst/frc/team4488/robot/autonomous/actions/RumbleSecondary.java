package org.usfirst.frc.team4488.robot.autonomous.actions;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import org.usfirst.frc.team4488.robot.operator.Controllers;
import org.usfirst.frc.team4488.robot.operator.Logging;

public class RumbleSecondary implements Action {
  Logging logger = Logging.getInstance();

  private int updateCycles = 0;

  @Override
  public boolean isFinished() {
    return updateCycles > 20;
  }

  @Override
  public void update() {
    updateCycles++;
  }

  @Override
  public void done() {
    Controllers.getInstance().m_secondary.setRumble(RumbleType.kLeftRumble, 0);
    logger.writeToLogFormatted(this, "done()");
  }

  @Override
  public void start() {
    logger.writeToLogFormatted(this, "start()");
    Controllers.getInstance().m_secondary.setRumble(RumbleType.kLeftRumble, 1);
  }
}
