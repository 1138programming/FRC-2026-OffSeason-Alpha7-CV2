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
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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
    canBus = new CANBus(kSHOOTER_MOTOR_CANBUS);
    shooterMotor1 = new TalonFX(kSHOOTER_MOTOR_1_ID, canBus);
    shooterMotor2 = new TalonFX(kSHOOTER_MOTOR_2_ID, canBus);
    shooterMotor3 = new TalonFX(kSHOOTER_MOTOR_3_ID, canBus);

    configureShooterMotor();

    shooterMotor2.setControl(new Follower(kSHOOTER_MOTOR_1_ID, kSHOOTER_MOTOR_2_ALIGNMENT));
    shooterMotor3.setControl(new Follower(kSHOOTER_MOTOR_1_ID, kSHOOTER_MOTOR_3_ALIGNMENT));

    shooterMotorPowerRequest = new DutyCycleOut(0.0);
    shooterVelocityRequest = new VelocityVoltage(0.0).withSlot(0);
    shooterStopRequest = new NeutralOut();

    flywheelVelocitySignal = shooterMotor1.getVelocity();
    flywheelVelocitySignal.setUpdateFrequency(100);
  }

  private void configureShooterMotor() {
    final TalonFXConfiguration config = new TalonFXConfiguration()
      .withMotorOutput(
        new MotorOutputConfigs()
          .withInverted(InvertedValue.Clockwise_Positive)
          .withNeutralMode(NeutralModeValue.Coast)
      )
      .withFeedback(
        new FeedbackConfigs()
          .withSensorToMechanismRatio(1 / kFLYWHEEL_GEAR_RATIO)
      )
      .withSlot0(
        new Slot0Configs()
          .withKS(kS)
          .withKV(kV)
          .withKP(kP)
          .withKI(kI)
          .withKD(kD)
      )
      .withCurrentLimits(
        //if experiencing power issues, check here
        new CurrentLimitsConfigs()
          .withStatorCurrentLimit(kSHOOTER_STATOR_CURRENT_LIMIT)
          .withStatorCurrentLimitEnable(true)
          .withSupplyCurrentLimit(kSHOOTER_SUPPLY_CURRENT_LIMIT)
          .withSupplyCurrentLimitEnable(true) 
      );
      
    tryUntilOk(5, () -> shooterMotor1.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> shooterMotor2.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> shooterMotor3.getConfigurator().apply(config, 0.25));
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
    return targetRPM > 0.0 && Math.abs(getFlywheelRPM() - targetRPM) <= kFLYWHEEL_TOLERANCE_RPM;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
