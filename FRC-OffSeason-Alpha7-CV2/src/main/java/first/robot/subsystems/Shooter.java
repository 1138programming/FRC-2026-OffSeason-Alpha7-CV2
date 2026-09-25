// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.subsystems;

import org.wpilib.command2.SubsystemBase;
import org.wpilib.hardware.bus.CAN;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.controller.PIDController;
import static first.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

public class Shooter extends SubsystemBase {
  TalonFX shooterMotor1;
  TalonFX shooterMotor2;
  TalonFX shooterMotor3;
  PIDController shooterRPMPIDController;
  private CANBus canBus;

  /** Creates a new Shooter. */
  public Shooter() {
    canBus = new CANBus(kshooterMotorCANbus);
    shooterMotor1 = new TalonFX(kshooterMotor1ID, canBus);
    shooterMotor2 = new TalonFX(kshooterMotor2ID, canBus);
    shooterMotor3 = new TalonFX(kshooterMotor3ID, canBus);
    shooterRPMPIDController = new PIDController(kP, kI, kD);
  }

  public void spinFlywheelMotors(double power){}
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
