package org.usfirst.frc.team4488.robot.systems;

// Unit test framework imports
import static org.junit.Assert.*;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.powermock.core.classloader.annotations.PrepareForTest;
import org.powermock.core.classloader.annotations.SuppressStaticInitializationFor;
import org.powermock.api.mockito.PowerMockito;
import org.powermock.modules.junit4.PowerMockRunner;

// Robot-related imports
import org.usfirst.frc.team4488.lib.util.NavX;
import org.usfirst.frc.team4488.robot.operator.Controllers;
import org.usfirst.frc.team4488.robot.operator.Logging;
import org.usfirst.frc.team4488.lib.util.PreferencesParser;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Solenoid;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import org.usfirst.frc.team4488.robot.sensors.AnalogBounceback;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@RunWith(PowerMockRunner.class)
@PrepareForTest({
  Drive.class,
  NavX.class,
  Controllers.class,
  SmartDrive.class,
  Logging.class,
  PreferencesParser.class,
  NetworkTableInstance.class,
  SmartDashboard.class
})
@SuppressStaticInitializationFor("edu.wpi.first.wpilibj.smartdashboard.SmartDashboard")
public class DriveTest {
  WPI_TalonSRX mockTalon = PowerMockito.mock(WPI_TalonSRX.class);
  WPI_VictorSPX mockVictor = PowerMockito.mock(WPI_VictorSPX.class);
  Solenoid mockSolenoid = PowerMockito.mock(Solenoid.class);
  Controllers mockXbox = PowerMockito.mock(Controllers.class);
  PreferencesParser mockPrefs = PowerMockito.mock(PreferencesParser.class);
  Logging mockLog = PowerMockito.mock(Logging.class);
  AnalogBounceback mockBounceback = PowerMockito.mock(AnalogBounceback.class);
  SensorCollection mockSensorCollection = PowerMockito.mock(SensorCollection.class);
  DigitalInput mockDigitalInput = PowerMockito.mock(DigitalInput.class);
  // @TODO why can't I mock an actual NavX device, i.e. the AHRS
  NavX mockNavX = PowerMockito.mock(NavX.class);

  @Test
  public void testConstructor() throws Exception {
    PowerMockito.mockStatic(NetworkTableInstance.class, invocationOnMock -> null);
    NetworkTableInstance ni = PowerMockito.mock(NetworkTableInstance.class);
    PowerMockito.when(NetworkTableInstance.getDefault()).thenReturn(ni);
    PowerMockito.mockStatic(SmartDashboard.class);
    PowerMockito.whenNew(WPI_TalonSRX.class).withAnyArguments().thenReturn(mockTalon);
    PowerMockito.whenNew(WPI_VictorSPX.class).withAnyArguments().thenReturn(mockVictor);
    PowerMockito.whenNew(Solenoid.class).withAnyArguments().thenReturn(mockSolenoid);
    PowerMockito.whenNew(NavX.class).withAnyArguments().thenReturn(mockNavX);
    PowerMockito.when(mockTalon.getSensorCollection()).thenReturn(mockSensorCollection);
    PowerMockito.whenNew(Controllers.class).withAnyArguments().thenReturn(mockXbox);
    PowerMockito.whenNew(PreferencesParser.class).withAnyArguments().thenReturn(mockPrefs);
    PowerMockito.when(mockPrefs.getDouble("DriveUSP")).thenReturn(0.0);
    PowerMockito.whenNew(Logging.class).withAnyArguments().thenReturn(mockLog);
    PowerMockito.whenNew(AnalogBounceback.class).withAnyArguments().thenReturn(mockBounceback);
    PowerMockito.whenNew(DigitalInput.class).withAnyArguments().thenReturn(mockDigitalInput);

    Drive drive = new Drive();
    assertNotNull(drive);
  }
}
