// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.commands.manualAutons;

import static first.robot.Constants.ActiveFloorConstants.kFLOOR_POWER_INWARD;
import static first.robot.Constants.ShooterConstants.kShootRPM;

import org.wpilib.command2.Commands;
import org.wpilib.command2.SequentialCommandGroup;

import first.robot.commandGroups.throughputGroups.runFloorIndexerShooter;
import first.robot.commands.DriveCommands;
import first.robot.commands.SpinShooterAtRPMCommand;
import first.robot.commands.ActiveFloor.FloorInward;
import first.robot.commands.IndexerCommands.IndexInCommand;
import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.ActiveFloor;
import first.robot.subsystems.Indexer;
import first.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class manualBackupAndShootCommand extends SequentialCommandGroup {
  /** Creates a new manualBackupAndShootCommand. */
  public manualBackupAndShootCommand(ActiveFloor floor, Indexer indexer, Shooter shooter, Drive drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(//early rev
        Commands.race(
            new SpinShooterAtRPMCommand(shooter, kShootRPM),
            DriveCommands.runVelocity(drive, -1.214, 0, 0).withTimeout(1.3)
        ),

        //dump it all -> check if automatically stops
        Commands.sequence(
            new SpinShooterAtRPMCommand(shooter, kShootRPM).withTimeout(5),
            Commands.race(            new IndexInCommand(indexer),
            new FloorInward(floor, kFLOOR_POWER_INWARD),
            new SpinShooterAtRPMCommand(shooter, kShootRPM),
            Commands.waitSeconds(6)
            )
        )
    );
  }
}
