package org.usfirst.frc.team4488.robot.autonomous.actions;

import java.util.ArrayList;
import java.util.List;
import org.usfirst.frc.team4488.robot.operator.Logging;

/** Executes one action at a time. Useful as a member of {@link ParallelAction} */
public class SeriesAction implements Action {
  Logging logger = Logging.getInstance();

  private Action mCurAction;
  private final ArrayList<Action> mRemainingActions;

  public SeriesAction(List<Action> actions) {
    mRemainingActions = new ArrayList<>(actions.size());

    for (Action action : actions) {
      mRemainingActions.add(action);
    }

    mCurAction = null;
  }

  @Override
  public boolean isFinished() {
    return mRemainingActions.isEmpty() && mCurAction == null;
  }

  @Override
  public void start() {
    logger.writeToLogFormatted(this, "start()");
  }

  @Override
  public void update() {
    if (mCurAction == null) {
      if (mRemainingActions.isEmpty()) {
        return;
      }

      mCurAction = mRemainingActions.remove(0);
      mCurAction.start();
    }

    mCurAction.update();

    if (mCurAction.isFinished()) {
      mCurAction.done();
      mCurAction = null;
    }
  }

  @Override
  public void done() {
    logger.writeToLogFormatted(this, "done()");
  }
}
