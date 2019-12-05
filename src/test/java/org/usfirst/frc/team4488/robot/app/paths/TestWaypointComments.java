package org.usfirst.frc.team4488.robot.app.paths;

import static org.junit.Assert.*;

import org.junit.Test;
import org.usfirst.frc.team4488.lib.util.app.control.Path;
import org.usfirst.frc.team4488.lib.util.waypointCommentParser;

public class TestWaypointComments {

  public <T extends PathContainer> void testClass(Class<T> cls) {
    PathContainer actualDrivePath;
    try {
      actualDrivePath = cls.newInstance();
      Path actual = actualDrivePath.buildPath();
      assertNotNull(actualDrivePath);
      Path expected = waypointCommentParser.readWaypointComments(actualDrivePath.getClass());
      assertEquals(actual.toString(), expected.toString());
    } catch (InstantiationException | IllegalAccessException e) {
      e.printStackTrace();
    }
  }

  @Test
  public void Straight() {
    testClass(Straight.class);
  }
}
