// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.subsystems;

import org.wpilib.command2.SubsystemBase;
import static first.robot.Constants.LimelightConstants.*;

public class Limelight extends SubsystemBase {
  private Limelight limelight;
  /** Creates a new Limelight. */
  public Limelight() {
    limelight = new Limelight(klimelightName);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
