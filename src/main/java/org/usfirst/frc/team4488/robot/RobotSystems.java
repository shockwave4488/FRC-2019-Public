package org.usfirst.frc.team4488.robot;

import java.util.ArrayList;
import org.usfirst.frc.team4488.robot.systems.Subsystem;
import org.usfirst.frc.team4488.robot.systems.drive.DriveBase;

public class RobotSystems {
  public DriveBase drive;
  public ArrayList<Subsystem> otherSystems;

  public RobotSystems(DriveBase drive, ArrayList<Subsystem> otherSystems) {
    this.drive = drive;
    this.otherSystems = otherSystems;
  }
}
