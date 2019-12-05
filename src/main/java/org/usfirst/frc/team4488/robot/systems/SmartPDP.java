package org.usfirst.frc.team4488.robot.systems;

import edu.wpi.first.wpilibj.PowerDistributionPanel;

public class SmartPDP extends PowerDistributionPanel {

  public static SmartPDP sInstance = null;
  /** @return an instance of smart PDP */
  public static SmartPDP getInstance() {
    if (sInstance == null) {
      sInstance = new SmartPDP();
    }

    return sInstance;
  }
}
