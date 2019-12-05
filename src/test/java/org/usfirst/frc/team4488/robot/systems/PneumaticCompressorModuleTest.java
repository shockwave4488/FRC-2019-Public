package org.usfirst.frc.team4488.robot.systems;

// Unit test framework imports
import static org.junit.Assert.*;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.powermock.core.classloader.annotations.PrepareForTest;
import org.powermock.api.mockito.PowerMockito;
import org.powermock.modules.junit4.PowerMockRunner;

// Robot-related imports
import edu.wpi.first.wpilibj.Compressor;

@RunWith(PowerMockRunner.class)
@PrepareForTest({PneumaticCompressorModule.class})
public class PneumaticCompressorModuleTest {

  Compressor mockCompressor = PowerMockito.mock(Compressor.class);

  @Test
  public void testConstructor() throws Exception {
    PowerMockito.whenNew(Compressor.class).withAnyArguments().thenReturn(mockCompressor);

    PneumaticCompressorModule pneumaticCompressorModule = new PneumaticCompressorModule();
    assertNotNull(pneumaticCompressorModule);
  }
}
