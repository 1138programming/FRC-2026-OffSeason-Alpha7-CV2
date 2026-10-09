// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.littletonrobotics.junction.networktables.LoggedNetworkChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.command2.button.JoystickButton;
import org.wpilib.command2.button.Trigger;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;

import com.ctre.phoenix6.hardware.CANrange;

import first.robot.commands.DriveCommands;
import first.robot.commands.ActiveFloor.FloorInward;
import first.robot.commands.ActiveFloor.FloorOutward;
import first.robot.commands.IntakeCommands.IntakePivotDeploy;
import first.robot.commands.IntakeCommands.IntakePivotStow;
import first.robot.commands.IntakeCommands.IntakeRollerIn;
import first.robot.commands.IntakeCommands.IntakeRollerOut;
import first.robot.commands.SpinShooterAtRPMCommand;
import first.robot.commands.IndexerCommands.IndexInCommand;
import first.robot.commands.IndexerCommands.IndexOutCommand;
import first.robot.commands.IndexerCommands.IndexUntilBallIn;
import first.robot.subsystems.ActiveFloor;
import first.robot.subsystems.CANRangeSensor;
import first.robot.subsystems.Indexer;
import first.robot.subsystems.Intake;
import first.robot.subsystems.Shooter;
import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.drive.DriveConstants;
import first.robot.subsystems.drive.GyroIO;
import first.robot.subsystems.drive.GyroIOOnboardIMU;
import first.robot.subsystems.drive.ModuleIO;
import first.robot.subsystems.drive.ModuleIOSim;
import first.robot.subsystems.drive.ModuleIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedNetworkChooser;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import first.robot.subsystems.drive.GyroIOPigeon2;
import first.robot.subsystems.drive.ModuleIO;
import first.robot.subsystems.drive.ModuleIOSim;
import first.robot.subsystems.drive.ModuleIOTalonFX;
import first.robot.subsystems.vision.Vision;
import first.robot.subsystems.vision.VisionIO;
import first.robot.subsystems.vision.VisionIOLimelight;

import static first.robot.Constants.ActiveFloorConstants.*;
import static first.robot.Constants.OperatorConstants.*;
import static first.robot.Constants.ShooterConstants.*;
import static first.robot.Constants.LimelightConstants.*;
import static first.robot.Constants.FieldConstants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Indexer indexer;
  private final Shooter shooter;
  private final Intake intake;
  private final ActiveFloor activeFloor;

  private final CANRangeSensor indexerCANRange;
  private final Vision vision;

  // Controller. CommandGamepad uses controller-agnostic names: faceDown/faceRight/faceLeft/faceUp
  // are A/B/X/Y on an Xbox pad.
  private final CommandGamepad controller = new CommandGamepad(0);
  private final CommandGamepad compStreamDeck = new CommandGamepad(1);

  private final IndexInCommand indexInCommand;
  private final IndexOutCommand indexOutCommand;
  private final IndexUntilBallIn indexUntilBallInCommand;

  private boolean fieldRelative = true;

  // Dashboard inputs
  private final LoggedNetworkChooser<Command> autoChooser;
  private final LoggedNetworkNumber shooterTuningRPM =
      new LoggedNetworkNumber("/Tuning/Shooter RPM", kShootRPM);

  public Trigger
    logitechButtonA,
    logitechButtonB,
    logitechButtonY,
    logitechButtonX,
    logitechButtonLB,
    logitechButtonRB,
    logitechButtonLT,
    logitechButtonRT,
    logitechButtonBack,
    logitechButtonStart;


  public Trigger 
    compStreamDeck1, 
    compStreamDeck2, 
    compStreamDeck3,
    compStreamDeck4, 
    compStreamDeck5, 
    compStreamDeck6,
    compStreamDeck7, 
    compStreamDeck8, 
    compStreamDeck9,
    compStreamDeck10, 
    compStreamDeck11, 
    compStreamDeck12,
    compStreamDeck13, 
    compStreamDeck14, 
    compStreamDeck15,
    compStreamDeck16, 
    compStreamDeck17, 
    compStreamDeck18,
    compStreamDeck19;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    indexer = new Indexer();
    shooter = new Shooter();
    intake = new Intake();
    activeFloor = new ActiveFloor();

    indexerCANRange = new CANRangeSensor();

    indexInCommand = new IndexInCommand(indexer);
    indexOutCommand = new IndexOutCommand(indexer);
    indexUntilBallInCommand = new IndexUntilBallIn(indexerCANRange, indexer);


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

    vision = switch (Constants.getMode()) {
        case REAL -> new Vision(new VisionIOLimelight(klimelightName, kROBOT_TO_CAMERA),
            drive::addVisionMeasurement, drive::getRotation, drive::getYawRateRadPerSec);
        default -> new Vision(new VisionIO() {},
            drive::addVisionMeasurement, drive::getRotation, drive::getYawRateRadPerSec);
    };

    

    // Set up auto routines
    autoChooser = new LoggedNetworkChooser<>("/SmartDashboard/Auto Choices");
    autoChooser.addDefault("None", Commands.none());

    // Set up characterization routines
    autoChooser.add(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.add(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));

    logitechButtonA = controller.button(kLOGITECH_BUTTON_A);
    logitechButtonB = controller.button(kLOGITECH_BUTTON_B);
    logitechButtonY = controller.button(kLOGITECH_BUTTON_Y);
    logitechButtonX = controller.button(kLOGITECH_BUTTON_X);
    logitechButtonLB = controller.button(kLOGITECH_BUTTON_LB);
    logitechButtonRB = controller.button(kLOGITECH_BUTTON_RB);
    logitechButtonLT = controller.button(kLOGITECH_BUTTON_LT);
    logitechButtonRT = controller.button(kLOGITECH_BUTTON_RT);
    logitechButtonBack = controller.button(kLOGITECH_BUTTON_BACK);
    logitechButtonStart = controller.button(kLOGITECH_BUTTON_START);

    compStreamDeck1 = compStreamDeck.button(1);
    compStreamDeck2 = compStreamDeck.button(2);
    compStreamDeck3 = compStreamDeck.button(3);
    compStreamDeck4 = compStreamDeck.button(4);
    compStreamDeck5 = compStreamDeck.button(5);
    compStreamDeck6 = compStreamDeck.button(6);
    compStreamDeck7 = compStreamDeck.button(7);
    compStreamDeck8 = compStreamDeck.button(8);
    compStreamDeck9 = compStreamDeck.button(9);
    compStreamDeck10 = compStreamDeck.button(10);
    compStreamDeck11 = compStreamDeck.button(11);
    compStreamDeck12 = compStreamDeck.button(12);
    compStreamDeck13 = compStreamDeck.button(13);
    compStreamDeck14 = compStreamDeck.button(14);
    compStreamDeck15 = compStreamDeck.button(15);
    compStreamDeck16 = compStreamDeck.button(16);
    compStreamDeck17 = compStreamDeck.button(17);
    compStreamDeck18 = compStreamDeck.button(18);
    compStreamDeck19 = compStreamDeck.button(19);

    // Configure the button bindings
    configureButtonBindings();
  }

  /** Maps driver inputs to commands. */
  private void configureButtonBindings() {
    // Default command, field-relative drive with the red-alliance perspective flipped 180 degrees.
    // Forward on the stick maps to -X and left maps to -Y before the alliance adjustment
    // (sign convention from kramer/driver_trials, tested on the robot).
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            controller::getLeftY,
            controller::getLeftX,
            controller::getRightX,
            () -> fieldRelative));

    // Toggle field/robot relative on right stick press without interrupting the drive command
    controller.rightStick().onTrue(Commands.runOnce(() -> fieldRelative = !fieldRelative));

    // Lock to 0 degrees while A is held
    controller
        .faceDown()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                controller::getLeftY,
                controller::getLeftX,
                () -> Rotation2d.ZERO));

    // Switch to X pattern when X is pressed
    controller.faceLeft().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset the gyro heading to 0 degrees when 'Back' is pressed
    logitechButtonBack
        .onTrue(
            Commands.runOnce(
                    () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.ZERO)),
                    drive)
                .ignoringDisable(true));


    //shooter buttons (hold to spin, release to stop)
    // Right trigger: spin up to the preset shot RPM
    controller.rightTrigger().whileTrue(new SpinShooterAtRPMCommand(shooter, kShootRPM));
    // Left trigger: spin up to the RPM set on the dashboard at /Tuning/Shooter RPM
    controller.leftTrigger().whileTrue(new SpinShooterAtRPMCommand(shooter, shooterTuningRPM));
    // Right bumper: open-loop test power, for checking direction before trusting the gains
    controller
        .rightBumper()
        .whileTrue(
            Commands.startEnd(
                () -> shooter.spinFlywheelMotors(kTestDutyCycle), shooter::stopFlywheelMotors, shooter));

    //indexer buttons (moved off LT/RB, which the shooter uses)
    controller.faceUp().whileTrue(indexInCommand);
    controller.leftBumper().whileTrue(indexOutCommand);
    controller.faceRight().whileTrue(indexUntilBallInCommand);

    //intake buttons
    controller.dpadUp().onTrue(new IntakePivotDeploy(intake));
    controller.dpadDown().onTrue(new IntakePivotStow(intake));
    controller.dpadLeft().whileTrue(new IntakeRollerIn(intake));
    controller.dpadRight().whileTrue(new IntakeRollerOut(intake));

    //floor buttons
    controller.start().whileTrue(new FloorInward(activeFloor, kFLOOR_POWER_INWARD));
    controller.leftStick().whileTrue(new FloorOutward(activeFloor, kFLOOR_POWER_OUTWARD));
    
  }

  /** Field-relative heading that points the robot's front at our alliance's HUB. */
  private Rotation2d headingToGoal() {
    boolean isRed = MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.RED;
    Translation2d goal = isRed ? kRED_HUB : kBLUE_HUB;
    // getAngle() is empty only when the robot is exactly on the goal; hold heading then.
    return goal.minus(drive.getPose().getTranslation()).getAngle().orElse(drive.getRotation());
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
