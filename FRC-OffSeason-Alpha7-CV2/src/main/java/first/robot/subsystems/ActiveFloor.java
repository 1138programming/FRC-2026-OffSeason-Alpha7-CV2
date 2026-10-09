// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.subsystems;

import org.littletonrobotics.junction.ConsoleSource.Systemcore;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.hardware.expansionhub.ExpansionHubMotor.NeutralMode;
import org.wpilib.hardware.rotation.DutyCycle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

import static first.robot.Constants.ActiveFloorConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ActiveFloor extends SubsystemBase {

  private TalonFX floorRollerMotor;
  private final DutyCycleOut mfloorRunRequest;
  private final NeutralOut mstopRequest;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> appliedVoltsSignal;
  private final StatusSignal<Current> statorCurrentSignal;
  private double requestedPower = 0.0;
  /** Creates a new ActiveFloor. */
  public ActiveFloor() {
    floorRollerMotor = new TalonFX(kFLOOR_ROLLER_MOTOR_ID,new CANBus(CANPort.CAN_S0));

    mfloorRunRequest = new DutyCycleOut(0.0);
    mstopRequest = new NeutralOut();

    configureFloorRollerMotor();

    velocitySignal = floorRollerMotor.getVelocity(false);
    appliedVoltsSignal = floorRollerMotor.getMotorVoltage(false);
    statorCurrentSignal = floorRollerMotor.getStatorCurrent(false);
    BaseStatusSignal.setUpdateFrequencyForAll(50, velocitySignal, appliedVoltsSignal, statorCurrentSignal);
  }

  private void configureFloorRollerMotor() {
    final TalonFXConfiguration config = new TalonFXConfiguration()
      .withMotorOutput(
        new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake)
      );

      floorRollerMotor.getConfigurator().apply(config);
  }

  public void setFloorPower(double power) {
    requestedPower = power;
    floorRollerMotor.setControl(mfloorRunRequest.withOutput(power));
  }

  public void stopfloor() {
    requestedPower = 0.0;
    floorRollerMotor.setControl(mstopRequest);
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(velocitySignal, appliedVoltsSignal, statorCurrentSignal);

    Logger.recordOutput("ActiveFloor/RequestedPower", requestedPower);
    Logger.recordOutput("ActiveFloor/RPM", velocitySignal.getValueAsDouble() * 60.0);
    Logger.recordOutput("ActiveFloor/AppliedVolts", appliedVoltsSignal.getValueAsDouble());
    Logger.recordOutput("ActiveFloor/StatorCurrentAmps", statorCurrentSignal.getValueAsDouble());
  }
}
