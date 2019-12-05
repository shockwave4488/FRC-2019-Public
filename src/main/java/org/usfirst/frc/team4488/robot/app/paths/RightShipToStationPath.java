package org.usfirst.frc.team4488.robot.app.paths;

import java.util.ArrayList;
import org.usfirst.frc.team4488.lib.util.app.control.Path;
import org.usfirst.frc.team4488.lib.util.app.math.RigidTransform2d;
import org.usfirst.frc.team4488.lib.util.app.math.Translation2d;
import org.usfirst.frc.team4488.robot.app.RobotState;
import org.usfirst.frc.team4488.robot.app.paths.PathBuilder.Waypoint;

public class RightShipToStationPath implements PathContainer {

  @Override
  public Path buildPath() {
    Translation2d pos =
        RobotState.getInstance().getLatestFieldToVehicle().getValue().getTranslation();

    ArrayList<Waypoint> points = new ArrayList<Waypoint>();
    points.add(new Waypoint(pos.x(), pos.y(), 0, 140));
    points.add(new Waypoint(pos.x() - 30, pos.y(), 20, 140)); // 20
    points.add(new Waypoint(pos.x() - 130, pos.y() - 73, 10, 140)); // 10
    points.add(new Waypoint(pos.x() - 160, pos.y() - 80, 0, 140));
    points.add(new Waypoint(pos.x() - 180, pos.y() - 80, 0, 140));

    return PathBuilder.buildPathFromWaypoints(points);
  }

  @Override
  public RigidTransform2d getStartPose() {
    RigidTransform2d pose = RobotState.getInstance().getLatestFieldToVehicle().getValue();
    return pose;
  }

  @Override
  public boolean isReversed() {
    return true;
  }
}
