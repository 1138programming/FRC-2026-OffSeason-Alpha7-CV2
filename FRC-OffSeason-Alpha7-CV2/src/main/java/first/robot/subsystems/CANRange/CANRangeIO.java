package first.robot.subsystems.CANRange;

import com.ctre.phoenix6.signals.MeasurementHealthValue;
import org.littletonrobotics.junction.AutoLog;

public interface CANRangeIO {
  @AutoLog
  public static class CANRangeIOInputs {
    public boolean connected = false;
    public double distanceMeters = 0.0;
    public double distanceStdDevMeters = 0.0;
    public boolean isDetected = false;
    public double signalStrength = 0.0;
    public double ambientSignal = 0.0;
    public MeasurementHealthValue health = MeasurementHealthValue.Bad;
    public double measurementTimeSecs = 0.0;
    public double supplyVoltage = 0.0;
  }

  public default void updateInputs(CANRangeIOInputs inputs) {}
}