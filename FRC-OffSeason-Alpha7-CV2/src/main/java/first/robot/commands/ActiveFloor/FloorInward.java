// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.commands.ActiveFloor;

import static first.robot.Constants.ActiveFloorConstants.kFLOOR_POWER_INWARD;

import org.wpilib.command2.Command;

import first.robot.subsystems.ActiveFloor;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FloorInward extends Command {

  private ActiveFloor activeFloor;

  /** Creates a new FloorInward. */
  public FloorInward(ActiveFloor activeFloor) {
    this.activeFloor = activeFloor;
    addRequirements(activeFloor);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    activeFloor.setFloorPower(kFLOOR_POWER_INWARD);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    activeFloor.stopfloor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
