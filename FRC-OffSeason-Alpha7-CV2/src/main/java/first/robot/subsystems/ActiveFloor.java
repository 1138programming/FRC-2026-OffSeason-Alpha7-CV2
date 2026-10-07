// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.subsystems;

import org.littletonrobotics.junction.ConsoleSource.Systemcore;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.hardware.expansionhub.ExpansionHubMotor.NeutralMode;
import org.wpilib.hardware.rotation.DutyCycle;

import static first.robot.Constants.ActiveFloorConstants.*;

import com.ctre.phoenix6.CANBus;
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
  /** Creates a new ActiveFloor. */
  public ActiveFloor() {
    floorRollerMotor = new TalonFX(kFLOOR_ROLLER_MOTOR_ID,new CANBus(CANPort.CAN_S0));

    mfloorRunRequest = new DutyCycleOut(0.0);
    mstopRequest = new NeutralOut();

    configureFloorRollerMotor();
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
    floorRollerMotor.setControl(mfloorRunRequest.withOutput(power));
  }

  public void stopfloor() {
    floorRollerMotor.setControl(mstopRequest);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
