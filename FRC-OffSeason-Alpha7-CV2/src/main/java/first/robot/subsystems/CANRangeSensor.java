// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.subsystems;

import static first.robot.Constants.CANRangeConstants.*;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.SubsystemBase;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANrange;


public class CANRangeSensor extends SubsystemBase {
  CANrange CANRangeSensor;
  /** Creates a new CANRange. */
  public CANRangeSensor() {
    CANRangeSensor = new CANrange(kCANRangeID, new CANBus(kS0));
  }

  public double getDistanceFromObjectCM(){
    return CANRangeSensor.getDistance().getValueAsDouble() * kMtoCM;
  }

  public boolean IsBallInIndexer(){
    return getDistanceFromObjectCM() < kNoFuelDistance;
  
  }

  @Override
  public void periodic() {
    Logger.recordOutput("CANRange/DistanceCM", getDistanceFromObjectCM());
    Logger.recordOutput("CANRange/BallInIndexer", IsBallInIndexer());
    Logger.recordOutput("CANRange/Connected", CANRangeSensor.isConnected());
  }
}
