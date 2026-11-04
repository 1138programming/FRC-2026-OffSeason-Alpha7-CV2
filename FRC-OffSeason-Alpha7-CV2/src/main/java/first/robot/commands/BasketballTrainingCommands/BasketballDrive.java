// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.commands.BasketballTrainingCommands;

import org.wpilib.command2.Command;
import first.robot.subsystems.drive.Drive;
import first.robot.commands.DriveCommands;
import org.wpilib.system.Timer;



/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BasketballDrive extends Command {
  private Drive drive;
  private double seconds;
  private boolean forward;
  private Timer timer;

  /** Creates a new IndexCommand. */
  public BasketballDrive(Drive drive, double seconds, boolean forward) {
    this.drive = drive;
    this.seconds = seconds;
    this.forward = forward;
    this.timer = new Timer();

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DriveCommands.runVelocity(drive, forward ? 1:0, 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      DriveCommands.runVelocity(drive, 0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > seconds;
  }
}
