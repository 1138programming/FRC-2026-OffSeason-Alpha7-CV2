package first.robot.subsystems.CANRange;

import static first.robot.Constants.CANRangeConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.MeasurementHealthValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import first.robot.subsystems.drive.ModuleIOTalonFX;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Time;
import org.wpilib.units.measure.Voltage;

public class CANRangeIOReal implements CANRangeIO {
  private final CANrange canrange = new CANrange(kCANrangeID, new CANBus(CANPort.CAN_S0));

  private final StatusSignal<Distance> distance = canrange.getDistance();
  private final StatusSignal<Distance> distanceStdDev = canrange.getDistanceStdDev();
  private final StatusSignal<Boolean> isDetected = canrange.getIsDetected();
  private final StatusSignal<Double> signalStrength = canrange.getSignalStrength();
  private final StatusSignal<Double> ambientSignal = canrange.getAmbientSignal();
  private final StatusSignal<MeasurementHealthValue> health = canrange.getMeasurementHealth();
  private final StatusSignal<Time> measurementTime = canrange.getMeasurementTime();
  private final StatusSignal<Voltage> supplyVoltage = canrange.getSupplyVoltage();

  public CANRangeIOReal() {
    var config = new CANrangeConfiguration();
    config.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
    config.ProximityParams.ProximityThreshold = 0.4;
    config.ProximityParams.ProximityHysteresis = 0.01;
    config.ProximityParams.MinSignalStrengthForValidMeasurement = 2500;
    ModuleIOTalonFX.tryUntilOk(5, () -> canrange.getConfigurator().apply(config, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        distance,
        distanceStdDev,
        isDetected,
        signalStrength,
        ambientSignal,
        health,
        measurementTime,
        supplyVoltage);
  }

  @Override
  public void updateInputs(CANRangeIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                distance,
                distanceStdDev,
                isDetected,
                signalStrength,
                ambientSignal,
                health,
                measurementTime,
                supplyVoltage)
            .isOK();
    inputs.distanceMeters = distance.getValueAsDouble();
    inputs.distanceStdDevMeters = distanceStdDev.getValueAsDouble();
    inputs.isDetected = isDetected.getValue();
    inputs.signalStrength = signalStrength.getValue();
    inputs.ambientSignal = ambientSignal.getValue();
    inputs.health = health.getValue();
    inputs.measurementTimeSecs = measurementTime.getValueAsDouble();
    inputs.supplyVoltage = supplyVoltage.getValueAsDouble();
  }
}
