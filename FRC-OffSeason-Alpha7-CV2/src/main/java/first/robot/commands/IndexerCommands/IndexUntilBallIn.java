// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.commands.IndexerCommands;

import org.wpilib.command2.Command;

import first.robot.subsystems.CANRangeSensor;
import first.robot.subsystems.Indexer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IndexUntilBallIn extends Command {
  CANRangeSensor canRangeSensor;
  Indexer indexer;

  /** Creates a new IndexUntilBallIn. */
  public IndexUntilBallIn(CANRangeSensor canRangeSensor, Indexer indexer) {
    this.canRangeSensor = canRangeSensor;
    this.indexer = indexer;
    addRequirements(canRangeSensor, indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    indexer.spinIndexerIn();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.stopIndexerMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return canRangeSensor.IsBallInIndexer();
  }
}
