package org.usfirst.frc.team4488.robot.app.paths;

import java.util.ArrayList;
import org.usfirst.frc.team4488.lib.util.app.control.Path;
import org.usfirst.frc.team4488.lib.util.app.math.RigidTransform2d;
import org.usfirst.frc.team4488.lib.util.app.math.Translation2d;
import org.usfirst.frc.team4488.robot.app.RobotState;
import org.usfirst.frc.team4488.robot.app.paths.PathBuilder.Waypoint;
import org.usfirst.frc.team4488.robot.systems.SubsystemManager;
import org.usfirst.frc.team4488.robot.systems.SubsystemManager.Sides;
import org.usfirst.frc.team4488.robot.Constants;

public class StationToRightShip implements PathContainer {

  @Override
  public Path buildPath() {
    Translation2d pos =
        RobotState.getInstance().getLatestFieldToVehicle().getValue().getTranslation();

    ArrayList<Waypoint> points = new ArrayList<Waypoint>();
    /*
    points.add(new Waypoint(pos.x(), pos.y(), 0, 120));
    points.add(new Waypoint(pos.x() + 80, pos.y(), 10, 120));
    points.add(new Waypoint(pos.x() + 282, pos.y() - 102, 20, 120));
    */

    points.add(new Waypoint(pos.x() + 30, pos.y(), 0, 140));
    points.add(new Waypoint(pos.x() + 227, pos.y() + 42, 0, 140));
    double offset = (SubsystemManager.getInstance().getSelectedSide()) == Sides.Red ? Constants.rightRedEndOffset : Constants.rightBlueEndOffset;
    points.add(new Waypoint(pos.x() + 267 + offset, pos.y() + 56, 30, 90));
    points.add(new Waypoint(pos.x() + 267 + offset, pos.y() + 91, 0, 90));

    return PathBuilder.buildPathFromWaypoints(points);
  }

  @Override
  public RigidTransform2d getStartPose() {
    RigidTransform2d pose = RobotState.getInstance().getLatestFieldToVehicle().getValue();
    return pose;
  }

  @Override
  public boolean isReversed() {
    return false;
  }
}
