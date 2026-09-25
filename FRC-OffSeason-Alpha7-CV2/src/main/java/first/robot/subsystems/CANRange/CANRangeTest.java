package first.robot.subsystems.CANRange;

import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

public class CANRangeTest extends SubsystemBase {
  private final CANRangeIO io;
  private final CANRangeIOInputsAutoLogged inputs = new CANRangeIOInputsAutoLogged();
  private final Alert disconnectedAlert =
      new Alert("canRangeDisconnected", "CANrange disconnected!", Level.HIGH);

  public CANRangeTest(CANRangeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("CANRange", inputs);
    disconnectedAlert.set(!inputs.connected);

    Logger.recordOutput("CANRange/DistanceInches", inputs.distanceMeters * 39.3701);

    Telemetry.log("CANRange/Connected", inputs.connected);
    Telemetry.log("CANRange/Distance (m)", inputs.distanceMeters);
    Telemetry.log("CANRange/Distance (in)", inputs.distanceMeters * 39.3701);
    Telemetry.log("CANRange/Detected", inputs.isDetected);
    Telemetry.log("CANRange/Signal Strength", inputs.signalStrength);
    Telemetry.log("CANRange/Ambient", inputs.ambientSignal);
    Telemetry.log("CANRange/Health", inputs.health.toString());
    Telemetry.log("CANRange/Std Dev (m)", inputs.distanceStdDevMeters);
  }

  public boolean isDetected() {
    return inputs.isDetected;
  }

  public double getDistanceMeters() {
    return inputs.distanceMeters;
  }
}
