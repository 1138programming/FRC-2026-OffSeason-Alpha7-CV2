// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.commandGroups.throughputGroups;

import static first.robot.Constants.ShooterConstants.kReverseRPM;

import org.wpilib.command2.ParallelCommandGroup;

import first.robot.commands.SpinShooterAtRPMCommand;
import first.robot.commands.IndexerCommands.IndexOutCommand;
import first.robot.subsystems.Indexer;
import first.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class runIndexerShooterReverse extends ParallelCommandGroup {
  /** Creates a new runIndexerShooterReverse. */
  public runIndexerShooterReverse(Indexer indexer, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IndexOutCommand(indexer),
      new SpinShooterAtRPMCommand(shooter, kReverseRPM)
    );
  }
}
