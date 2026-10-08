// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.subsystems;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.math.util.Units;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.system.RobotController;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;
import static first.robot.Constants.ShooterConstants.*;
import static first.robot.subsystems.drive.ModuleIOTalonFX.tryUntilOk;
import first.robot.Constants;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Shooter extends SubsystemBase {
  private TalonFX shooterMotor1;
  private TalonFX shooterMotor2;
  private TalonFX shooterMotor3;
  private DutyCycleOut shooterMotorPowerRequest;
  private VelocityVoltage shooterVelocityRequest;
  private NeutralOut shooterStopRequest;
  private StatusSignal<AngularVelocity> flywheelVelocitySignal;
  private StatusSignal<Voltage> appliedVoltsSignal;
  private StatusSignal<Current> statorCurrentSignal;
  private CANBus canBus;
  private double targetRPM = 0.0;

  private final Alert duplicateIDAlert =
      new Alert("Shooter", "Shooter CAN IDs are not unique, followers disabled. Set IDs in Constants.", Level.HIGH);

  // Only used in simulation, where the leader's voltage drives all three motors
  private FlywheelSim flywheelSim;

  /** Creates a new Shooter. */
  public Shooter() {
    canBus = new CANBus(kshooterMotorCANbus);
    shooterMotor1 = new TalonFX(kshooterMotor1ID, canBus);
    shooterMotor2 = new TalonFX(kshooterMotor2ID, canBus);
    shooterMotor3 = new TalonFX(kshooterMotor3ID, canBus);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = kshooterMotor1Inverted; // followers ignore this and use their alignment
    config.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimitAmps;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = kSupplyCurrentLimitAmps;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.Feedback.SensorToMechanismRatio = 1.0 / kFlywheelGearRatio; // 3 motor rotations per 2 flywheel rotations
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    tryUntilOk(5, () -> shooterMotor1.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> shooterMotor2.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> shooterMotor3.getConfigurator().apply(config, 0.25));

    // With placeholder IDs every TalonFX object points at the same device, so making the "followers"
    // follow motor 1 would tell it to follow itself. Skip that until real IDs are set.
    boolean idsUnique = kshooterMotor1ID != kshooterMotor2ID
        && kshooterMotor1ID != kshooterMotor3ID
        && kshooterMotor2ID != kshooterMotor3ID;
    duplicateIDAlert.set(!idsUnique);
    if (idsUnique) {
      shooterMotor2.setControl(new Follower(kshooterMotor1ID, kshooterMotor2Alignment));
      shooterMotor3.setControl(new Follower(kshooterMotor1ID, kshooterMotor3Alignment));
    }

    shooterMotorPowerRequest = new DutyCycleOut(0.0);
    shooterVelocityRequest = new VelocityVoltage(0.0).withSlot(0);
    shooterStopRequest = new NeutralOut();

    flywheelVelocitySignal = shooterMotor1.getVelocity();
    appliedVoltsSignal = shooterMotor1.getMotorVoltage();
    statorCurrentSignal = shooterMotor1.getStatorCurrent();
    flywheelVelocitySignal.setUpdateFrequency(100);
    BaseStatusSignal.setUpdateFrequencyForAll(50, appliedVoltsSignal, statorCurrentSignal);

    if (Constants.getMode() == Constants.Mode.SIM) {
      DCMotor gearbox = DCMotor.getKrakenX60(3);
      flywheelSim = new FlywheelSim(
          Models.flywheelFromPhysicalConstants(gearbox, kFlywheelMOI, 1.0 / kFlywheelGearRatio), gearbox);
    }
  }

  public void spinFlywheelMotors(double power){
    targetRPM = 0.0;
    shooterMotor1.setControl(shooterMotorPowerRequest.withOutput(power));
  }

  public void stopFlywheelMotors(){
    targetRPM = 0.0;
    shooterMotor1.setControl(shooterStopRequest);
  }

  /** Flywheel speed in RPM, updated once per loop in {@link #periodic()}. */
  public double getFlywheelRPM(){
    return flywheelVelocitySignal.getValueAsDouble() * 60.0;
  }

  public void spinFlywheelAtRPM(double rpm){
    targetRPM = rpm;
    shooterMotor1.setControl(shooterVelocityRequest.withVelocity(rpm / 60.0));
  }

  public boolean isFlywheelAtSpeed(){
    return targetRPM > 0.0 && Math.abs(getFlywheelRPM() - targetRPM) <= kFlywheelToleranceRPM;
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(flywheelVelocitySignal, appliedVoltsSignal, statorCurrentSignal);

    Logger.recordOutput("Shooter/RPM", getFlywheelRPM());
    Logger.recordOutput("Shooter/TargetRPM", targetRPM);
    Logger.recordOutput("Shooter/AtSpeed", isFlywheelAtSpeed());
    Logger.recordOutput("Shooter/AppliedVolts", appliedVoltsSignal.getValueAsDouble());
    Logger.recordOutput("Shooter/StatorCurrentAmps", statorCurrentSignal.getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    if (flywheelSim == null) {
      return;
    }
    var simState = shooterMotor1.getSimState();
    simState.setSupplyVoltage(RobotController.getBatteryVoltage());

    flywheelSim.setInputVoltage(simState.getMotorVoltage());
    flywheelSim.update(Constants.kLOOPPERIODSECS);

    // TalonFX sim expects rotor (motor-side) units
    double rotorRPS = Units.radiansToRotations(flywheelSim.getAngularVelocity()) / kFlywheelGearRatio;
    simState.setRotorVelocity(rotorRPS);
    simState.addRotorPosition(rotorRPS * Constants.kLOOPPERIODSECS);
  }
}
