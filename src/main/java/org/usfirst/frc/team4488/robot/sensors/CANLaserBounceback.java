package org.usfirst.frc.team4488.robot.sensors;

import edu.wpi.first.hal.can.CANJNI;
import edu.wpi.first.wpilibj.Timer;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class CANLaserBounceback {

  public enum Ranges {
    SHORT(0),
    MEDIUM(1),
    LONG(2);

    public byte value;

    private Ranges(int value) {
      this.value = (byte) value;
    }
  }

  // Message ID's are from the sensor data sheet in google drive
  // Recieved messages
  private static final int HEARTBEAT_MESSAGE_ID = 0x18F0FF00;
  private static final int CALIBRATION_STATE_MESSAGE_ID = 0x0CF91200;
  private static final int MEASUREMENT_MESSAGE_ID = 0x0CF91000;
  private static final int MEASUREMENT_QUALITY_MESSAGE_ID = 0x0CF91100;

  // Sent messages
  private static final int RANGING_CONFIG_MESSAGE_ID = 0x0CF91300;
  private static final int DEVICE_CONFIG_MESSAGE_ID = 0x0CAAFFF9;
  private static final int DEVICE_IDENTIFICATION_MESSAGE_ID = 0x0CAAFFF9;

  private final int deviceID;

  // should never be less than the measurement period
  private static final double DEAD_TIME_THRESHOLD = 0.5; // seconds
  private double lastMeasurementTime = 0;
  private byte[] lastHeartbeat = new byte[]{0,0,0,0,0,0,0,0};
  private byte[] lastMeasurement = new byte[]{0,0,0,0,0,0,0,0};
  private byte[] lastMeasurementQuality = new byte[]{0,0,0,0,0,0,0,0};

  private Ranges range = Ranges.SHORT;
  private int measurementPeriod = 100;
  private static final int MINIMUM_MEASUREMENT_PERIOD = 24;
  private static final int MAXIMUM_MEASUREMENT_PERIOD = 1000;

  public CANLaserBounceback(int deviceID) {
    this.deviceID = deviceID;
  }

  private static byte[] getMessage(int messageID, int deviceID) throws Exception {
    ByteBuffer idBuffer = ByteBuffer.allocateDirect(4);
    idBuffer.order(ByteOrder.LITTLE_ENDIAN);
    idBuffer.asIntBuffer().put(0, messageID | deviceID);

    ByteBuffer timestampBuffer = ByteBuffer.allocateDirect(4);
    timestampBuffer.order(ByteOrder.LITTLE_ENDIAN);
    timestampBuffer.asIntBuffer().put(0, 0x00000000);

    byte[] data =
        CANJNI.FRCNetCommCANSessionMuxReceiveMessage(
            idBuffer.asIntBuffer(), 0x1fffffff, timestampBuffer);

    return data;
  }

  private static void sendMessage(int messageID, byte[] message) {
    CANJNI.FRCNetCommCANSessionMuxSendMessage(messageID, message, 0);
  }

  private byte[] getHeartbeatMessage() {
    try {
      byte[] received = getMessage(HEARTBEAT_MESSAGE_ID, deviceID);
      lastHeartbeat = received;
      return received;
    } catch (Exception e) {
      return lastHeartbeat;
    }
  }

  private byte[] getMeasurementMessage() {
    try {
      byte[] received = getMessage(MEASUREMENT_MESSAGE_ID, deviceID);
      lastMeasurement = received;
      lastMeasurementTime = Timer.getFPGATimestamp();
      return received;
    } catch (Exception e) {
      return lastMeasurement;
    }
  }

  private byte[] getMeasurementQualityMessage() {
    try {
      byte[] received = getMessage(MEASUREMENT_QUALITY_MESSAGE_ID, deviceID);
      lastMeasurementQuality = received;
      return received;
    } catch (Exception e) {
      return lastMeasurementQuality;
    }
  }

  private void sendRangeConfig() {
    byte[] message = new byte[3];
    message[0] = range.value;
    message[1] = (byte) (measurementPeriod & 0xff);
    message[2] = (byte) ((measurementPeriod >> 8) & 0xff);

    sendMessage(RANGING_CONFIG_MESSAGE_ID | deviceID, message);
  }

  public int getSerialNumber() {
    byte[] heartbeat = getHeartbeatMessage();
    int serialNumber =
        (heartbeat[1] & 0xff) | ((heartbeat[2] & 0xff) << 8) | ((heartbeat[3] & 0xff) << 16);

    return serialNumber;
  }

  public int getFirmwareVersion() {
    byte[] heartbeat = getHeartbeatMessage();
    int firmware = (heartbeat[6] & 0xff) | ((heartbeat[7] & 0xff) << 8);

    return firmware;
  }

  public int getDistanceMillimeters() {
    byte[] measurement = getMeasurementMessage();
    int measured = (measurement[0] & 0xff) | ((measurement[1] & 0xff) << 8);

    return measured;
  }

  public boolean isValid() {
    byte[] measurement = getMeasurementMessage();
    boolean validStatus = measurement[2] == 0;
    double timeSinceLastMessage = Timer.getFPGATimestamp() - lastMeasurementTime;

    return validStatus && (timeSinceLastMessage < DEAD_TIME_THRESHOLD);
  }

  public double getMeasurementStandardDeviation() {
    byte[] measurementQuality = getMeasurementQualityMessage();
    double standardDeviation =
        (measurementQuality[4] & 0xff)
            | ((measurementQuality[5] & 0xff) << 8)
            | ((measurementQuality[6] & 0xff) << 16)
            | ((measurementQuality[7] & 0xff) << 24);
    standardDeviation /= 65536; // Conversion to millimeters from data sheet

    return standardDeviation;
  }

  public void setRange(Ranges range) {
    this.range = range;
    sendRangeConfig();
  }

  public void setMeasurementPeriod(int period) {
    this.measurementPeriod =
        Math.max(MINIMUM_MEASUREMENT_PERIOD, Math.min(MAXIMUM_MEASUREMENT_PERIOD, period));
    sendRangeConfig();
  }

  public void lightLED() {
    lightLED(getSerialNumber());
  }

  public static void lightLED(int serialNumber) {
    byte[] message = new byte[6];
    message[0] = 0x0D; // fixed, from data sheet
    message[1] = (byte) (serialNumber & 0xff);
    message[2] = (byte) ((serialNumber >> 8) & 0xff);
    message[3] = (byte) ((serialNumber >> 16) & 0xff);
    message[4] = 0x10; // fixed, from data sheet
    message[5] = 0x01; // fixed, from data sheet

    sendMessage(DEVICE_IDENTIFICATION_MESSAGE_ID, message);
  }

  public static void setID(int serialNumber, int newID) {
    byte[] message = new byte[7];
    message[0] = 0x0C; // fixed, from data sheet
    message[1] = (byte) (serialNumber & 0xff);
    message[2] = (byte) ((serialNumber >> 8) & 0xff);
    message[3] = (byte) ((serialNumber >> 16) & 0xff);
    message[4] = 0x10; // fixed, from data sheet
    message[5] = 0x01; // fixed, from data sheet
    message[6] = (byte) (newID & 0xff);

    sendMessage(DEVICE_CONFIG_MESSAGE_ID, message);
  }
}
