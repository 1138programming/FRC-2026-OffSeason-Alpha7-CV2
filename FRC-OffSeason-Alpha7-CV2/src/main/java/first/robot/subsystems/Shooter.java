// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.subsystems;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.units.measure.AngularVelocity;
import static first.robot.Constants.ShooterConstants.*;
import static first.robot.subsystems.drive.ModuleIOTalonFX.tryUntilOk;
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
  private CANBus canBus;
  private double targetRPM = 0.0;

  /** Creates a new Shooter. */
  public Shooter() {
    canBus = new CANBus(kshooterMotorCANbus);
    shooterMotor1 = new TalonFX(kshooterMotor1ID, canBus);
    shooterMotor2 = new TalonFX(kshooterMotor2ID, canBus);
    shooterMotor3 = new TalonFX(kshooterMotor3ID, canBus);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast; 
    config.Feedback.SensorToMechanismRatio = 1.0 / kFlywheelGearRatio; // 3 motor rotations per 2 flywheel rotations
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    tryUntilOk(5, () -> shooterMotor1.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> shooterMotor2.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> shooterMotor3.getConfigurator().apply(config, 0.25));

    shooterMotor2.setControl(new Follower(kshooterMotor1ID, kshooterMotor2Alignment));
    shooterMotor3.setControl(new Follower(kshooterMotor1ID, kshooterMotor3Alignment));

    shooterMotorPowerRequest = new DutyCycleOut(0.0);
    shooterVelocityRequest = new VelocityVoltage(0.0).withSlot(0);
    shooterStopRequest = new NeutralOut();

    flywheelVelocitySignal = shooterMotor1.getVelocity();
    flywheelVelocitySignal.setUpdateFrequency(100);
  }

  public void spinFlywheelMotors(double power){
    targetRPM = 0.0;
    shooterMotor1.setControl(shooterMotorPowerRequest.withOutput(power));
  }

  public void stopFlywheelMotors(){
    targetRPM = 0.0;
    shooterMotor1.setControl(shooterStopRequest);
  }

  public double getFlywheelRPM(){
    
    return flywheelVelocitySignal.refresh().getValueAsDouble() * 60.0;
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
    // This method will be called once per scheduler run
  }
}
