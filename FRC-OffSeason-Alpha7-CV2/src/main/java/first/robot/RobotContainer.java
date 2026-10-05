// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import first.robot.commands.DriveCommands;
import first.robot.commands.IntakeCommands.IntakePivotDeploy;
import first.robot.commands.IntakeCommands.IntakePivotForward;
import first.robot.commands.IntakeCommands.IntakePivotReverse;
import first.robot.commands.IntakeCommands.IntakePivotStow;
import first.robot.commands.IntakeCommands.IntakeRollerIn;
import first.robot.commands.IntakeCommands.IntakeRollerOut;
import first.robot.subsystems.Intake;
import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.drive.DriveConstants;
import first.robot.subsystems.drive.GyroIO;
import first.robot.subsystems.drive.GyroIOOnboardIMU;
import first.robot.subsystems.drive.GyroIOPigeon2;
import first.robot.subsystems.drive.ModuleIO;
import first.robot.subsystems.drive.ModuleIOSim;
import first.robot.subsystems.drive.ModuleIOTalonFX;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.networktables.LoggedNetworkChooser;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Intake intake;

  // Controller. CommandGamepad uses controller-agnostic names: faceDown/faceRight/faceLeft/faceUp
  // are A/B/X/Y on an Xbox pad.
  private final CommandGamepad controller = new CommandGamepad(0);

  // Dashboard inputs
  private final LoggedNetworkChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    intake = new Intake();

    switch (Constants.getMode()) {
      case REAL ->
          // Real robot, instantiate hardware IO implementations
          drive =
              new Drive(
                  new GyroIOOnboardIMU(),
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
    // Default command, normal field-relative drive.
    // +X on the field is away from the driver station and +Y is to the left, so forward on the
    // stick (which reads negative) maps to +X and left (also negative) maps to +Y.
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));
    
    // System.out.println("Hello New Drive");
    // drive.setDefaultCommand(
    //     DriveCommands.joystickDrive(
    //         drive,
    //         () -> 1,
    //         () -> 0,
    //         () -> 0));

    // Lock to 0 degrees while A is held
    controller
        .faceDown()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.ZERO));

    // Switch to X pattern when X is pressed
    controller.faceLeft().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset the gyro heading to 0 degrees when B is pressed
    controller
        .faceRight()
        .onTrue(
            Commands.runOnce(
                    () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.ZERO)),
                    drive)
                .ignoringDisable(true));

    // Intake rollers: hold right trigger to intake, left trigger to outtake
    controller.rightTrigger().whileTrue(new IntakeRollerIn(intake));
    controller.leftTrigger().whileTrue(new IntakeRollerOut(intake));

    // Intake pivot: right bumper deploys, left bumper stows
    controller.rightBumper().onTrue(new IntakePivotDeploy(intake));
    controller.leftBumper().onTrue(new IntakePivotStow(intake));

    // Manual pivot override: hold d-pad up/down
    controller.dpadUp().whileTrue(new IntakePivotForward(intake));
    controller.dpadDown().whileTrue(new IntakePivotReverse(intake));
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
