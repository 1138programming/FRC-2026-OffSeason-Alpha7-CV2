// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.commandGroups.intake;

import org.wpilib.command2.SequentialCommandGroup;

import first.robot.commands.IntakeCommands.IntakePivotDeploy;
import first.robot.commands.IntakeCommands.IntakeRollerIn;
import first.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class intakeDeployThenIntake extends SequentialCommandGroup {
  /** Creates a new intakeDeployThenIntake. */
  public intakeDeployThenIntake(Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakePivotDeploy(intake).withTimeout(3),
      new IntakeRollerIn(intake)
    );
  }
}
