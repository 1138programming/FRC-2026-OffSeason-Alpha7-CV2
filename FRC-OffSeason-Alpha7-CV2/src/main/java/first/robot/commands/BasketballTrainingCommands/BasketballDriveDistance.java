// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.commands.BasketballTrainingCommands;

import org.wpilib.command2.Command;
import org.wpilib.command2.ParallelDeadlineGroup;
import org.wpilib.command2.WaitCommand;

import first.robot.subsystems.drive.Drive;
import first.robot.commands.DriveCommands;



/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BasketballDriveDistance extends ParallelDeadlineGroup {

  Drive drive;

  public BasketballDriveDistance(Drive drive, double meters, double speed) {
    this.drive = drive; 
    super(new WaitCommand(Math.abs(meters/speed)));

    addCommands(DriveCommands.runVelocity(drive, speed*Math.copySign(1, meters), 0, 0 ));
  }
}
