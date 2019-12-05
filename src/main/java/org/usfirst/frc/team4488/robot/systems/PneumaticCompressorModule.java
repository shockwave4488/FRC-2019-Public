package org.usfirst.frc.team4488.robot.systems;

import edu.wpi.first.wpilibj.Compressor;
import org.usfirst.frc.team4488.robot.loops.Looper;

public class PneumaticCompressorModule extends Subsystem {

  private static PneumaticCompressorModule sInstance = null;

  private Compressor pcm;

  public PneumaticCompressorModule() {
    pcm = new Compressor();
    pcm.start();
  }

  /** @return an instance of Pneumatic Compressor Module */
  public static PneumaticCompressorModule getInstance() {
    if (sInstance == null) {
      sInstance = new PneumaticCompressorModule();
    }

    return sInstance;
  }

  @Override
  public void stop() {}

  @Override
  public void updateSmartDashboard() {}

  @Override
  public void zeroSensors() {}

  @Override
  public void registerEnabledLoops(Looper enabledLooper) {}

  @Override
  public void updatePrefs() {}

  @Override
  public void writeToLog() {}

  @Override
  public void reset() {}
}
