package JavaRoboticsLib.ControlSystems;

import static org.junit.Assert.*;

import org.junit.Before;
import org.junit.Test;

public class SimPIDTest {
  SimPID pid;

  @Before
  public void setup() {
    pid = new SimPID(.15, 0, 0);
    pid.setDoneRange(1);
    pid.setErrorEpsilon(0);
  }

  @Test
  public void test() {
    pid.setDesiredValue(5);

    double power = pid.calcPID(0);
    System.out.println("Calculated Power: " + power);
    power = pid.calcPID(0);
    System.out.println("Calculated Power: " + power);
    power = pid.calcPID(0);
    System.out.println("Calculated Power: " + power);
    power = pid.calcPID(0);
    System.out.println("Calculated Power: " + power);
    power = pid.calcPID(0);
    System.out.println("Calculated Power: " + power);
  }
}
