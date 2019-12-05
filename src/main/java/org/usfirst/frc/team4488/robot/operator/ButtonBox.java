package org.usfirst.frc.team4488.robot.operator;

import edu.wpi.first.wpilibj.XboxController;

public class ButtonBox {
  public XboxController button_box;
  private static ButtonBox sInstance = null;

  /*
   * This is the ButtonBox class, which grabs raw booleans and number values
   * from an Xbox controller (that is how it is configured) and turns them into more obviously named
   * functions, which then are assigned to control different functions of the
   * robot.
   */

  public enum SecondaryState {
    manual,
    auto;
  }

  public SecondaryState secondaryState = SecondaryState.auto;

  public static ButtonBox getInstance() {
    if (sInstance == null) {
      sInstance = new ButtonBox();
    }
    return sInstance;
  }

  public ButtonBox() {
    button_box = new XboxController(2);
  }

  // ----------------------------------------
  // -----------GENERAL FUNCTIONS------------
  // ----------------------------------------

  // This function gets the button for the left ship 1st hole.
  public boolean getLShip1() {
    return button_box.getRawButton(6);
  }

  // This function gets the button for the left ship 2nd hole.
  public boolean getLShip2() {
    return button_box.getRawButton(7);
  }

  // This function gets the button for the left ship 3rd hole.
  public boolean getLShip3() {
    return button_box.getRawButton(8);
  }

  // This function gets the button for the left ship 4rd hole.
  public boolean getLShip4() {
    return button_box.getRawButton(9);
  }

  // This function gets the button for the right ship 1st hole.
  public boolean getRShip1() {
    return button_box.getRawButton(10);
  }

  // This function gets the button for the right ship 2nd hole.
  public boolean getRShip2() {
    return button_box.getRawButton(11);
  }

  // This function gets the button for the right ship 3rd hole.
  public boolean getRShip3() {
    return button_box.getRawButton(12);
  }

  // This function gets the button for the right ship 4th hole.
  public boolean getRShip4() {
    return button_box.getRawButton(13);
  }

  // This function gets the button for the left rocket 1st panel hole.
  public boolean getLRocketP1() {
    return button_box.getRawButton(3);
  }

  // This function gets the button for the left rocket 2nd panel hole.
  public boolean getLRocketP2() {
    return button_box.getRawButton(5);
  }

  // This function gets the button for the left rocket cargo hole.
  public boolean getLRocketC() {
    return button_box.getRawButton(4);
  }

  // This function gets the button for the right rocket 1st panel hole.
  public boolean getRRocketP1() {
    return button_box.getRawButton(14);
  }

  // This function gets the button for the right rocket 2nd panel hole.
  public boolean getRRocketP2() {
    return button_box.getRawButton(16);
  }

  // This function gets the button for the right rocket cargo hole.
  public boolean getRRocketC() {
    return button_box.getRawButton(15);
  }

  // This function gets the button for the 1st human player station.
  public boolean getHumanP1() {
    return button_box.getRawButton(1);
  }

  // This function gets the button for the 2nd human player station.
  public boolean getHumanP2() {
    return button_box.getRawButton(2);
  }
}
