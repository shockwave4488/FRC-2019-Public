package org.usfirst.frc.team4488.robot.autonomous.actions;

public class PrintToConsoleAction extends RunOnceAction implements Action {
  String debugMessage;

  public PrintToConsoleAction(String s) {
    debugMessage = s;
  }

  @Override
  public void runOnce() {
    System.out.println(debugMessage);
  }
}
