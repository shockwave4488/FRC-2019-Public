package org.usfirst.frc.team4488.robot.autonomous;

import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import org.usfirst.frc.team4488.lib.util.app.control.Path;
import org.usfirst.frc.team4488.lib.util.app.math.RigidTransform2d;
import org.usfirst.frc.team4488.lib.util.app.math.Rotation2d;
import org.usfirst.frc.team4488.lib.util.app.math.Translation2d;
import org.usfirst.frc.team4488.robot.Constants;
import org.usfirst.frc.team4488.robot.app.RobotState;
import org.usfirst.frc.team4488.robot.app.paths.PathBuilder;
import org.usfirst.frc.team4488.robot.app.paths.PathBuilder.Waypoint;
import org.usfirst.frc.team4488.robot.app.paths.PathContainer;
import org.usfirst.frc.team4488.robot.systems.Drive;

public class PathFinder {

  private static ArrayList<Node> nodes = new ArrayList<Node>();
  private static FieldMap field = FieldMap.getInstance();

  public static void init() {
    for (int x = 0; x < field.width; x += Constants.generatedPathResolution) {
      for (int y = 0; y < field.height; y += Constants.generatedPathResolution) {
        if (pointIsClear(x, y) && x >= 0 && x <= field.width && y >= 0 && y <= field.height)
          nodes.add(new Node(x, y));
      }
    }
  }

  private static class GeneratedPath implements PathContainer {
    private ArrayList<Waypoint> points = new ArrayList<Waypoint>();
    private boolean reversed = false;
    private Rotation2d heading;
    private final int speed = Constants.generatedPathSpeed;

    public GeneratedPath(Rotation2d heading) {
      this.heading = heading;
    }

    @Override
    public Path buildPath() {
      ArrayList<Waypoint> newList = new ArrayList<Waypoint>();
      for (int i = points.size() - 1; i >= 0; i--) {
        newList.add(points.get(i));
      }
      return PathBuilder.buildPathFromWaypoints(newList);
    }

    @Override
    public boolean isReversed() {
      Waypoint end = points.get(points.size() - 1);
      Waypoint almostEnd = points.get(points.size() - 2);
      double targetAngle =
          (360
                  - ((Math.atan2(
                              almostEnd.getTranslation().y() - end.getTranslation().y(),
                              almostEnd.getTranslation().x() - end.getTranslation().x())
                          / (2 * Math.PI))
                      * 360))
              % 360;
      double currentAngle = 360 - heading.getDegrees();
      return (Math.abs(targetAngle - currentAngle) > 90)
          && ((360 - targetAngle) + currentAngle > 90);
    }

    @Override
    public RigidTransform2d getStartPose() {
      return new RigidTransform2d(
          new Translation2d(
              points.get(points.size() - 1).getTranslation().x(),
              points.get(points.size() - 1).getTranslation().y()),
          heading);
    }

    public void setReversed(boolean reversed) {
      this.reversed = reversed;
    }

    public void addPoint(int x, int y) {
      // Y is inverted because PathFinder uses top left corner as (0,0) but APP uses bottom left for
      // (0,0)
      points.add(new Waypoint(x, field.height - y, 0, speed));
    }

    public ArrayList<Waypoint> getPoints() {
      return points;
    }
  }

  private static class Node {
    public int x;
    public int y;
    public int dist;
    public double fScore = 99999999;
    public double gScore = 99999999;
    public Node cameFrom;

    public Node(int x, int y, double gScore, double fScore) {
      this.x = x;
      this.y = y;
      this.gScore = gScore;
      this.fScore = fScore;
    }

    public Node(int x, int y) {
      this.x = x;
      this.y = y;
    }
  }

  // Uses the A* algorithm
  public static GeneratedPath generatePath(int startX, int startY, int endX, int endY) {
    if (nodes.size() == 0) init();

    // Y is inverted because PathFinder uses top left corner as (0,0) but APP uses bottom left for
    // (0,0)
    startY = field.height - startY;
    endY = field.height - endY;

    // Rounds end and start positions to nearest point
    double resolution = Constants.generatedPathResolution;
    startX = (int) (resolution * Math.round(startX / resolution));
    startY = (int) (resolution * Math.round(startY / resolution));
    endX = (int) (resolution * Math.round(endX / resolution));
    endY = (int) (resolution * Math.round(endY / resolution));

    ArrayList<Node> openSet = new ArrayList<Node>();
    ArrayList<Node> closedSet = new ArrayList<Node>();

    Node start = new Node(startX, startY, 0, heuristic(startX, startY, endX, endY));
    nodes.add(start);
    openSet.add(start);

    while (openSet.size() != 0) {
      Node current = getLowestF(openSet);
      if (current.x == endX && current.y == endY) return reconstruct(current);

      openSet.remove(current);
      closedSet.add(current);

      for (Node node : neighbors(nodes, current)) {
        if (closedSet.contains(node)) continue;
        double newGScore = current.gScore + heuristic(current.x, current.y, node.x, node.y);
        if (!openSet.contains(node)) {
          openSet.add(node);
        } else if (newGScore >= node.gScore) continue;

        node.cameFrom = current;
        node.gScore = newGScore;
        node.fScore = newGScore + heuristic(node.x, node.y, endX, endY);
      }
    }

    // If this is reached, there is no possible path;
    return null;
  }

  private static boolean pointIsClear(int x, int y) {
    for (int[] rect : field.obstacles) { // rect = [x1, y1, x2,  y2]
      int closestX = clamp(x, rect[0], rect[2]);
      int closestY = clamp(y, rect[1], rect[3]);

      double dist = Math.sqrt(Math.pow(x - closestX, 2) + Math.pow(y - closestY, 2));
      if (dist < Constants.generatedPathPadding) return false;
    }

    return true;
  }

  private static int clamp(int val, int min, int max) {
    return Math.min(Math.max(val, min), max);
  }

  private static GeneratedPath reconstruct(Node end) {
    GeneratedPath path = new GeneratedPath(Drive.getInstance().getAngleRotation2d());
    while (end != null) {
      path.addPoint(end.x, end.y);
      end = end.cameFrom;
    }

    return path;
  }

  private static Node getLowestF(ArrayList<Node> set) {
    Node lowest = set.get(0);
    for (Node node : set) {
      if (node.fScore < lowest.fScore) {
        lowest = node;
      }
    }

    return lowest;
  }

  private static double heuristic(double x1, double y1, double x2, double y2) {
    return Math.sqrt(Math.pow(x1 - x2, 2) + Math.pow(y1 - y2, 2));
  }

  public static ArrayList<Node> neighbors(ArrayList<Node> set, Node root) {
    ArrayList<Node> neighbors = new ArrayList<Node>();
    for (Node node : set) {
      if ((Math.abs(node.x - root.x) == Constants.generatedPathResolution
              && Math.abs(node.y - root.y) == 0)
          || (Math.abs(node.x - root.x) == 0
              && Math.abs(node.y - root.y) == Constants.generatedPathResolution)
          || (Math.abs(node.x - root.x) == Constants.generatedPathResolution
              && Math.abs(node.y - root.y) == Constants.generatedPathResolution)) {
        neighbors.add(node);
      }
    }

    return neighbors;
  }

  private double calcDist(int x1, int y1, int x2, int y2) {
    return Math.sqrt(Math.pow(x1 - x2, 2) + Math.pow(y1 - y2, 2));
  }

  // Auto zeroing stuff

  /** Should be called every time the robot finishes lining up at a vision target */
  public void zeroAtVisionTarget() {
    int[] closest = field.visionTargets.get(0);
    Translation2d currentPos =
        RobotState.getInstance().getLatestFieldToVehicle().getValue().getTranslation();
    int currX = (int) currentPos.x();
    int currY = (int) currentPos.y();
    for (int[] target : field.visionTargets) {
      target[1] = field.height - target[1];
      if (calcDist(currX, currY, target[0], target[1])
          < calcDist(currX, currY, closest[0], closest[1])) closest = target;
    }

    RobotState.getInstance()
        .reset(
            Timer.getFPGATimestamp(),
            new RigidTransform2d(
                new Translation2d(closest[0], field.height - closest[1]),
                Rotation2d.fromDegrees(Drive.getInstance().getAngle())));
  }
}
