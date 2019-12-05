package org.usfirst.frc.team4488.lib.util;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import org.usfirst.frc.team4488.lib.util.app.control.Path;
import org.usfirst.frc.team4488.robot.app.paths.PathBuilder;
import org.usfirst.frc.team4488.robot.app.paths.PathBuilder.Waypoint;

public class waypointCommentParser {
  public static Path readWaypointComments(Object objRef) {
    String pathToFile = objRef.toString();
    pathToFile = pathToFile.replace(".", "/");
    pathToFile = pathToFile.replaceFirst("class ", "");

    java.nio.file.Path currentRelativePath = java.nio.file.Paths.get("");
    String fullPath = currentRelativePath.toAbsolutePath().toString();
    fullPath += "/src/main/java/" + pathToFile + ".java";
    String rawText = "";
    try {
      FileReader fr = new FileReader(fullPath);
      int i;
      while ((i = fr.read()) != -1) rawText += (char) i;
      fr.close();
    } catch (FileNotFoundException e) {
      e.printStackTrace();
    } catch (IOException e) {
      e.printStackTrace();
    }
    int beginning = rawText.indexOf("WAYPOINT");
    int end = rawText.indexOf("\n", beginning);

    String waypointComment = rawText.substring(beginning, end);
    String strarr[] = waypointComment.split("position");
    // Now we have an array with N waypoints
    int maxWaypoints = 20;
    int numElements = 4;
    String stringWaypoints[][] = new String[maxWaypoints][numElements];
    int y = 0;
    for (String points : strarr) {
      int x = 0;

      for (String elements : points.split(":")) {
        switch (x) {
          case 2:
          case 4:
          case 5:
            stringWaypoints[y][x - 2] = elements.split(",")[0];
            break;
          case 3:
            stringWaypoints[y][x - 2] = elements.split("}")[0];
            break;
          default:
            break;
        }
        x++;
      }
      if (x > 2) {
        y++;
      }
    }
    // Now you can create Y waypoints
    ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
    for (int count = 0; count < y; count++) {
      sWaypoints.add(
          new Waypoint(
              Double.parseDouble(stringWaypoints[count][0]),
              Double.parseDouble(stringWaypoints[count][1]),
              Double.parseDouble(stringWaypoints[count][3]),
              Double.parseDouble(stringWaypoints[count][2])));
    }
    return PathBuilder.buildPathFromWaypoints(sWaypoints);
    // @TODO add is reversed
  }
}
