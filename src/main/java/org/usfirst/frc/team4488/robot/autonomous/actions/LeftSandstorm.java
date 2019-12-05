package org.usfirst.frc.team4488.robot.autonomous.actions;

import java.util.ArrayList;
import org.usfirst.frc.team4488.robot.app.paths.LeftShipToStationPath;
import org.usfirst.frc.team4488.robot.app.paths.StationToLeftShip;
import org.usfirst.frc.team4488.robot.autonomous.actions.SideCargoScoreLine.Targets;

public class LeftSandstorm implements Action {

  private Action routine;

  public LeftSandstorm() {
    ArrayList<Action> actions = new ArrayList<Action>();

    ArrayList<Action> scorePanelAndRotateTurret = new ArrayList<Action>();
    ArrayList<Action> rotateTurretAndWait = new ArrayList<Action>();
    rotateTurretAndWait.add(new TimerWait(750));
    rotateTurretAndWait.add(new RotateArmAction(30, 10));
    rotateTurretAndWait.add(new TurnTurretAction(90, 10));
    rotateTurretAndWait.add(new TimerWait(1));

    scorePanelAndRotateTurret.add(new SeriesAction(rotateTurretAndWait));

    ArrayList<Action> driveForwardsAndScore = new ArrayList<Action>();
    driveForwardsAndScore.add(new DriveStraightDistance(190, 1));
    driveForwardsAndScore.add(new LinePanelScore(true));

    scorePanelAndRotateTurret.add(new SeriesAction(driveForwardsAndScore));

    actions.add(new ParallelAction(scorePanelAndRotateTurret));

    ArrayList<Action> driveAndRotateTurret = new ArrayList<Action>();
    driveAndRotateTurret.add(new DrivePathUntilCamera(new LeftShipToStationPath()));

    ArrayList<Action> waitAndRotateTurret = new ArrayList<Action>();
    waitAndRotateTurret.add(new TimerWait(500));
    waitAndRotateTurret.add(new RotateArmAction(30, 10));
    waitAndRotateTurret.add(new TurnTurretAction(180, 10));
    waitAndRotateTurret.add(new ExtendArmAction(12.1, 3));
    waitAndRotateTurret.add(new RotateArmAction(-5, 5));

    driveAndRotateTurret.add(new SeriesAction(waitAndRotateTurret));

    actions.add(new ParallelAction(driveAndRotateTurret));

    actions.add(new CapturePanel(true));
    ArrayList<Action> driveAndSetupArm = new ArrayList<Action>();
    driveAndSetupArm.add(new RebuildAndDrivePathAction(new StationToLeftShip()));

    ArrayList<Action> setupArm = new ArrayList<Action>();
    setupArm.add(new ExtendArmAction(5.6, 1));
    setupArm.add(new RotateArmAction(20, 5));

    ArrayList<Action> waitAndSetupArm = new ArrayList<Action>();
    waitAndSetupArm.add(new RotateArmAction(20, 5));
    waitAndSetupArm.add(new ParallelAction(setupArm));

    driveAndSetupArm.add(new SeriesAction(waitAndSetupArm));
    driveAndSetupArm.add(new WaitAndTurnTurretToField(90, 5, 500));

    actions.add(new TimerWait(200));
    actions.add(new ParallelAction(driveAndSetupArm));
    actions.add(new SidePanelScore(Targets.LeftShip, true));

    routine = new SeriesAction(actions);
    // Drive to ship, score
  }

  @Override
  public void start() {
    routine.start();
  }

  @Override
  public void update() {
    routine.update();
  }

  @Override
  public void done() {
    routine.done();
  }

  @Override
  public boolean isFinished() {
    return routine.isFinished();
  }
}
