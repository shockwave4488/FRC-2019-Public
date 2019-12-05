package org.usfirst.frc.team4488.robot.autonomous;

/*
 * Adds radio buttons to the SmartDashboard interface that allow the user to select starting position
 * and desired type of auto routine. Decision logic is used to combine these choices with the
 * Field Management Data (FMS) to select the type of auto routine you want to run.
 */
public class AutoModeSelector {

  /*
   * returns the auto mode that you will run for your autonomous session
   */
  public static AutoModeBase getSelectedAutoMode() {
    return null; // TODO replace
  }

  /*
   * displays radio buttons on SmartDashboard. See determineAutoModeToUse() for details
   */
  public static void init() {}

  /*
   * used to display the auto mode details, what starting side we selected,
   * what the interpreted FMS data said about the ownership of switch and scale,
   * and finally what actual auto mode was selected.
   */
  private static void displayAutoModeDetails(AutoModeBase selectedAutoMode) {}
}
