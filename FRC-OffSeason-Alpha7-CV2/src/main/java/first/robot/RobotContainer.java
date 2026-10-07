// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import static first.robot.Constants.OperatorConstants.kLOGITECH_BUTTON_DPAD_UP;
import static first.robot.Constants.OperatorConstants.kLOGITECH_BUTTON_DPAD_DOWN;
import static first.robot.Constants.OperatorConstants.kLOGITECH_BUTTON_DPAD_LEFT;
import static first.robot.Constants.OperatorConstants.kLOGITECH_BUTTON_DPAD_RIGHT;
import static first.robot.Constants.OperatorConstants.kLOGITECH_BUTTON_Y;

import first.robot.commands.DriveCommands;
import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.drive.DriveConstants;
import first.robot.subsystems.drive.GyroIO;
import first.robot.subsystems.drive.GyroIOPigeon2;
import first.robot.subsystems.drive.ModuleIO;
import first.robot.subsystems.drive.ModuleIOSim;
import first.robot.subsystems.drive.ModuleIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedNetworkChooser;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandGamepad;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // This branch constructs only the drivetrain; mechanism hardware is inactive.
  private final Drive drive;
  private final CommandGamepad controller = new CommandGamepad(0);
  private final LoggedNetworkChooser<Command> autoChooser;
  private boolean fieldRelative = true;

  public RobotContainer() {
    switch (Constants.getMode()) {
      case REAL ->
          // Real robot, instantiate hardware IO implementations
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(DriveConstants.moduleConfigs[0]),
                  new ModuleIOTalonFX(DriveConstants.moduleConfigs[1]),
                  new ModuleIOTalonFX(DriveConstants.moduleConfigs[2]),
                  new ModuleIOTalonFX(DriveConstants.moduleConfigs[3]));

      case SIM ->
          // Sim robot, instantiate physics sim IO implementations. There is no Pigeon sim, so the
          // heading comes from the module kinematics instead.
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim());

      default ->
          // Replayed robot, disable IO implementations
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {});
    }

    // Set up auto routines
    autoChooser = new LoggedNetworkChooser<>("/SmartDashboard/Auto Choices");
    autoChooser.addDefault("None", Commands.none());

    // Set up characterization routines
    autoChooser.add(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.add(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));

    // Configure the button bindings
    configureButtonBindings();
  }

  /** Maps driver inputs to commands. */
  private void configureButtonBindings() {
    // Default command, field-relative drive with the red-alliance perspective flipped 180 degrees.
    // Forward on the stick maps to +X and left maps to +Y before the alliance adjustment.
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> controller.getRightX(),
            () -> fieldRelative));

    // Toggle once per Y press without interrupting the active drive command.
    controller.button(kLOGITECH_BUTTON_Y)
        .onTrue(Commands.runOnce(() -> fieldRelative = !fieldRelative));

    // D-pad test commands remain robot-relative at 0.5 m/s.
    // All buttons other than Y and the D-pad are intentionally unbound.
    double testVelocity = 0.5;
    controller.button(kLOGITECH_BUTTON_DPAD_UP)
        .whileTrue(DriveCommands.runVelocity(drive, -testVelocity, 0, 0));
    controller.button(kLOGITECH_BUTTON_DPAD_DOWN)
        .whileTrue(DriveCommands.runVelocity(drive, testVelocity, 0, 0));
    controller.button(kLOGITECH_BUTTON_DPAD_LEFT)
        .whileTrue(DriveCommands.runVelocity(drive, 0, -testVelocity, 0));
    controller.button(kLOGITECH_BUTTON_DPAD_RIGHT)
        .whileTrue(DriveCommands.runVelocity(drive, 0, testVelocity, 0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
