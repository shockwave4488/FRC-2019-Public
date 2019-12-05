package org.usfirst.frc.team4488.robot.operator;

import static org.junit.Assert.*;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import org.apache.commons.io.FileUtils;
import org.junit.Before;
import org.junit.Test;

public class LoggingTest {
  String fullPath;
  String testDirectory = "/build/.testLogger";

  @Before
  public void setup() {
    Path currentRelativePath = Paths.get("");
    fullPath = currentRelativePath.toAbsolutePath().toString();
    fullPath += testDirectory;
  }

  @Test
  public void testWriteToLog() {
    Logging logger = Logging.getInstance(fullPath);
    logger.writeRaw("test log print 1");
    logger.writeRaw("second log print");
    logger.flush();
    String actual = "";
    String expected = "test log print 1\nsecond log print\n";
    try {
      FileReader fr = new FileReader(logger.fullPath + "/main.txt");
      int i;
      while ((i = fr.read()) != -1) actual += (char) i;
      fr.close();
    } catch (FileNotFoundException e) {
      e.printStackTrace();
    } catch (IOException e) {
      e.printStackTrace();
    }
    assertEquals(actual, expected);
  }

  public void cleanUp() {
    try {
      FileUtils.deleteDirectory(new File(fullPath));
    } catch (IOException e) {
      e.printStackTrace();
    }
  }
}
