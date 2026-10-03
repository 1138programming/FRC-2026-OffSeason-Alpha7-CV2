// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.subsystems;

import org.wpilib.command2.SubsystemBase;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import static first.robot.Constants.IndexerConstants.*;

public class Indexer extends SubsystemBase {

  private TalonFX indexerLeft;
  private TalonFX indexerRight;
  CANBus canS0;
  private final DutyCycleOut indexerRunRequest;
  private final NeutralOut indexerStopRequest;

  /** Creates a new Indexer. */
  public Indexer() {
    canS0 = new CANBus(kS0);
    indexerLeft = new TalonFX(kindexerLeftID, canS0);
    indexerRight = new TalonFX(kindexerRightID, canS0);
    indexerRunRequest = new DutyCycleOut(0.0);
    indexerStopRequest = new NeutralOut();
  }

  public void spinIndexerMotors(double power){
    indexerLeft.setControl(indexerRunRequest.withOutput(power));
    indexerRight.setControl(indexerRunRequest.withOutput(power));
  }

  public void spinIndexerIn(){
    spinIndexerMotors(kindexInPower);
  }

  public void spinIndexerOut(){
    spinIndexerMotors(kindexOutPower);
  }

  public void stopIndexerMotors(){
    indexerLeft.setControl(indexerStopRequest);
    indexerRight.setControl(indexerStopRequest);
  }

  


}
