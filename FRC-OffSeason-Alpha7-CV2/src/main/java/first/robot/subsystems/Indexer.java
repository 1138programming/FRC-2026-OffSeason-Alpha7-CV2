// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.Follower;
import static first.robot.Constants.IndexerConstants.*;

public class Indexer extends SubsystemBase {

  private TalonFX indexerLeft;
  private TalonFX indexerRight;
  CANBus canS0;
  private final DutyCycleOut indexerRunRequest;
  private final NeutralOut indexerStopRequest;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> appliedVoltsSignal;
  private final StatusSignal<Current> statorCurrentSignal;
  private double requestedPower = 0.0;

  /** Creates a new Indexer. */
  public Indexer() {
    canS0 = new CANBus(kS0);
    indexerLeft = new TalonFX(kindexerLeftID, canS0);
    indexerRight = new TalonFX(kindexerRightID, canS0);
    indexerRunRequest = new DutyCycleOut(0.0);
    indexerStopRequest = new NeutralOut();
    
    indexerLeft.setControl(new Follower(kindexerRightID, kLeftMotorAlignment));

    velocitySignal = indexerRight.getVelocity(false);
    appliedVoltsSignal = indexerRight.getMotorVoltage(false);
    statorCurrentSignal = indexerRight.getStatorCurrent(false);
    BaseStatusSignal.setUpdateFrequencyForAll(50, velocitySignal, appliedVoltsSignal, statorCurrentSignal);
  }

  public void spinIndexerMotors(double power){
    requestedPower = power;
    indexerRight.setControl(indexerRunRequest.withOutput(power));
  }

  public void spinIndexerIn(){
    spinIndexerMotors(kindexInPower);
  }

  public void spinIndexerOut(){
    spinIndexerMotors(kindexOutPower);
  }

  public void stopIndexerMotors(){
    requestedPower = 0.0;
    indexerRight.setControl(indexerStopRequest);
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(velocitySignal, appliedVoltsSignal, statorCurrentSignal);

    Logger.recordOutput("Indexer/RequestedPower", requestedPower);
    Logger.recordOutput("Indexer/RPM", velocitySignal.getValueAsDouble() * 60.0);
    Logger.recordOutput("Indexer/AppliedVolts", appliedVoltsSignal.getValueAsDouble());
    Logger.recordOutput("Indexer/StatorCurrentAmps", statorCurrentSignal.getValueAsDouble());
  }
}
