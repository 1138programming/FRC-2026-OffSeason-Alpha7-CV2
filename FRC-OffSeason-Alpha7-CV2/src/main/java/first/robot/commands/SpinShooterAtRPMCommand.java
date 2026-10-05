// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.commands;

import java.util.function.DoubleSupplier;

import org.wpilib.command2.Command;

import first.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SpinShooterAtRPMCommand extends Command {
  Shooter shooter;
  DoubleSupplier RPM;
  /** Creates a new SpinShooterAtRPMCommand. */
  public SpinShooterAtRPMCommand(Shooter shooter, double RPM) {
    this(shooter, () -> RPM);
  }

  /** Spins at an RPM that is re-read every loop, e.g. from a dashboard number. */
  public SpinShooterAtRPMCommand(Shooter shooter, DoubleSupplier RPM) {
    this.shooter = shooter;
    this.RPM = RPM;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.spinFlywheelAtRPM(RPM.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopFlywheelMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
