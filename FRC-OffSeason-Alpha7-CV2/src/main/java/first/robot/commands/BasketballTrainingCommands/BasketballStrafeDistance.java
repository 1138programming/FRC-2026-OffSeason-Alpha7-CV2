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
public class BasketballStrafeDistance extends ParallelDeadlineGroup {

  Drive drive;
  double speed;

  public BasketballStrafeDistance(Drive drive, double meters, double speed) {
    this.drive = drive;
    this.speed = Math.clamp(speed, -2, 2);
    super(new WaitCommand(Math.abs(meters/speed)));

    addCommands(DriveCommands.runVelocity(drive, 0, speed*Math.copySign(1, meters), 0 ));
  }
}
