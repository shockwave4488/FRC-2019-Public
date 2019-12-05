package org.usfirst.frc.team4488.robot.autonomous;

import java.util.ArrayList;

public class FieldMap {
  public static FieldMap instance;

  public int width = 144;
  public int height = 95;

  public ArrayList<int[]> obstacles = new ArrayList<int[]>();

  // positions of all the vision targets on the field
  public ArrayList<int[]> visionTargets = new ArrayList<int[]>();

  public static FieldMap getInstance() {
    if (instance == null) instance = new FieldMap();
    return instance;
  }

  public FieldMap() {
    // Top left = (0,0)
    obstacles.add(new int[] {32, 3, 44, 15});
    obstacles.add(new int[] {65, 69, 86, 95});
    obstacles.add(new int[] {98, 5, 113, 20});

    // Approximations from chezy path, top left = (0,0)
    // Right now these are the ends of the white lines at every target
    visionTargets.add(new int[] {17, 299}); // Left loading station
    visionTargets.add(new int[] {17, 25}); // Right loading station
    visionTargets.add(new int[] {195, 296}); // Right rocket close side
    visionTargets.add(new int[] {228, 279}); // Right rocket center side
    visionTargets.add(new int[] {260, 296}); // Right rocket far side
    visionTargets.add(new int[] {195, 28}); // Left rocket close side
    visionTargets.add(new int[] {228, 45}); // Left rocket center side
    visionTargets.add(new int[] {260, 28}); // Left rocket far side
    visionTargets.add(new int[] {200, 174}); // Cargo ship R1
    visionTargets.add(new int[] {260, 209}); // Cargo ship R2
    visionTargets.add(new int[] {282, 209}); // Cargo ship R3
    visionTargets.add(new int[] {304, 209}); // Cargo ship R4
    visionTargets.add(new int[] {200, 150}); // Cargo ship L1
    visionTargets.add(new int[] {260, 115}); // Cargo ship L2
    visionTargets.add(new int[] {282, 115}); // Cargo ship L3
    visionTargets.add(new int[] {304, 115}); // Cargo ship L4
  }
}
