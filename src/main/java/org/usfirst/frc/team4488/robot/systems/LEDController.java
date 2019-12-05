package org.usfirst.frc.team4488.robot.systems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team4488.robot.loops.Loop;
import org.usfirst.frc.team4488.robot.loops.Looper;

public class LEDController extends Subsystem {

  public enum Color {
    Rainbow,
    Purple,
    White,
    Green,
    Default
  }

  private DigitalOutput ledDio0 = new DigitalOutput(2);
  private DigitalOutput ledDio1 = new DigitalOutput(3);
  private DigitalOutput ledDio2 = new DigitalOutput(4);

  private Color m_color = Color.Default;
  private Color blinkColor = Color.Default;
  private double blinkStart;
  private static int[] color_pattern = new int[3];

  private static LEDController sInstance;

  public static LEDController getInstance() {
    if (sInstance == null) {
      sInstance = new LEDController();
    }

    return sInstance;
  }

  private Loop mLoop =
      new Loop() {
        @Override
        public void onStart(double timestamp) {}

        @Override
        public void onLoop(double timestamp) {
          if ((Timer.getFPGATimestamp() * 1000) - blinkStart > 1500) {
            changeColor(m_color);
          } else {
            changeColor(blinkColor);
          }
        }

        @Override
        public void onStop(double timestamp) {}
      };

  /* *
   * Sets the color of the LED Controller.
   * @param color The specified color of the LED Controller
   * List of available colors can be seen in enum Color.
   * */
  public void setColor(Color color) {
    m_color = color;
    changeColor(color);
  }

  private void changeColor(Color color) {
    switch (color) {
      case Rainbow:
        // Rainbow binary pattern
        color_pattern[0] = 1;
        color_pattern[1] = 0;
        color_pattern[2] = 1;
        break;
      case Purple:
        // Purple binary pattern
        color_pattern[0] = 1;
        color_pattern[1] = 0;
        color_pattern[2] = 0;
        break;
      case Green:
        // Green binary pattern
        color_pattern[0] = 0;
        color_pattern[1] = 1;
        color_pattern[2] = 0;
        break;
      case White:
        // White binary pattern
        color_pattern[0] = 0;
        color_pattern[1] = 1;
        color_pattern[2] = 1;
        break;
      case Default:
        // Default binary pattern
        color_pattern[0] = 1;
        color_pattern[1] = 1;
        color_pattern[2] = 1;
        break;
      default:
        // Default binary patter
        color_pattern[0] = 1;
        color_pattern[1] = 1;
        color_pattern[2] = 1;
        break;
    }
    ledDio0.set(color_pattern[0] == 1);
    ledDio1.set(color_pattern[1] == 1);
    ledDio2.set(color_pattern[2] == 1);
  }

  public void blinkColor(Color color) {
    blinkColor = color;
    blinkStart = Timer.getFPGATimestamp() * 1000;
  }
  /**
   * Returns the current color instance of the LED Controller. Can be used as an additional checker
   * for if statements in routines.
   */
  public Color getColor() {
    return m_color;
  }

  @Override
  public void writeToLog() {}

  @Override
  public void updateSmartDashboard() {}

  @Override
  public void stop() {}

  @Override
  public void zeroSensors() {}

  @Override
  public void registerEnabledLoops(Looper enabledLooper) {
    enabledLooper.register(mLoop);
  }

  @Override
  public void updatePrefs() {}

  @Override
  public void reset() {}
}
