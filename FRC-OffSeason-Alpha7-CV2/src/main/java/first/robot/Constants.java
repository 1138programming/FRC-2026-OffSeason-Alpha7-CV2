// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.wpilib.framework.RobotBase;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on SystemCore. Change the value of {@link #kSIMMODE} to switch between "sim" (physics sim) and
 * "replay" (log replay from a file).
 */
public final class Constants {
  /**
   * Robot loop period. This is handed to {@code LoggedRobot} in {@link Robot}, so the value used
   * for velocity discretization and Phoenix status frame rates always matches the real loop rate.
   */
  public static final double kLOOPPERIODSECS = 0.02;

  /** Which physical robot the code is running on. Selects hardware IDs. */
  private static RobotType kROBOTTYPE = RobotType.DEVBOT;

  /** Enables tuning dashboard inputs. Must be false when merging. */
  public static final boolean kTUNINGMODE = false;

  /** Mode used when not running on real hardware. Set to REPLAY to replay a log instead. */
  public static final Mode kSIMMODE = Mode.SIM;

  @SuppressWarnings("resource")
  public static RobotType getRobot() {
    if (!disableHAL && RobotBase.isReal() && kROBOTTYPE == RobotType.SIMBOT) {
      new Alert(
              "invalidRobotType",
              "Invalid robot selected, using competition robot as default.",
              Level.MEDIUM)
          .set(true);
      kROBOTTYPE = RobotType.DEVBOT;
    }
    return kROBOTTYPE;
  }

  /**
   * Returns the current runtime mode. Real hardware is always {@link Mode#REAL}; off-robot this
   * follows {@link #kSIMMODE} so that the physics simulation actually runs by default.
   */
  public static Mode getMode() {
    return RobotBase.isReal() ? Mode.REAL : kSIMMODE;
  }

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public enum RobotType {
    DEVBOT,
    SIMBOT
  }

  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }

  /** Checks whether the correct robot is selected when deploying. */
  public static class CheckDeploy {
    public static void main(String... args) {
      if (kROBOTTYPE == RobotType.SIMBOT) {
        System.err.println("Cannot deploy, invalid robot selected: " + kROBOTTYPE);
        System.exit(1);
      }
    }
  }

  /** Checks that the default robot is selected and tuning mode is disabled. */
  public static class CheckPullRequest {
    public static void main(String... args) {
      if (kROBOTTYPE != RobotType.DEVBOT || kTUNINGMODE) {
        System.err.println("Do not merge, non-default constants are configured.");
        System.exit(1);
      }
    }
  }

  public static class OperatorConstants {
    //Logitech Button Constants
    public static final int kLOGITECH_BUTTON_A = 0;
    public static final int kLOGITECH_BUTTON_B = 1;
    public static final int kLOGITECH_BUTTON_Y = 3;
    public static final int kLOGITECH_BUTTON_X = 2;
    public static final int kLOGITECH_BUTTON_LB = 9;
    public static final int kLOGITECH_BUTTON_RB = 10;
    public static final int kLOGITECH_BUTTON_LT = 17;
    public static final int kLOGITECH_BUTTON_RT = 16;
    public static final int kLOGITECH_BUTTON_BACK = 4;
    public static final int kLOGITECH_BUTTON_START = 6;


  }

  private Constants() {}

  public static class IndexerConstants{
    public static final CANPort kS0 = CANPort.CAN_S1;
    public static final int kindexerLeftID = 23;
    public static final int kindexerRightID = 21;

    public static final double kindexInPower = 0.4;
    public static final double kindexOutPower = -0.4;

    public static final MotorAlignmentValue kLeftMotorAlignment = MotorAlignmentValue.Opposed;

  }

  public static class CANRangeConstants{
    public static final String kS0 = "*";
    public static final int kCANRangeID = 0;
    public static final double kMtoCM = 100.0;
    public static final double kNoFuelDistance = 38; //cm
  }

  public static class ShooterConstants{
    public static final CANPort kshooterMotorCANbus = CANPort.CAN_S1;

    // TODO: set real CAN IDs. While they match, followers are skipped and an alert is raised.
    public static final int kshooterMotor1ID = 22; // leader
    public static final int kshooterMotor2ID = 20;
    public static final int kshooterMotor3ID = 24;

    public static final double kFlywheelGearRatio = 2.0 / 3.0; // flywheel rotations per motor rotation

    // Flip if positive output spins the flywheel backwards (check with the open-loop test button)
    public static final InvertedValue kshooterMotor1Inverted = InvertedValue.CounterClockwise_Positive;

    public static final double kStatorCurrentLimitAmps = 80;
    public static final double kSupplyCurrentLimitAmps = 40;

    // Control setpoints
    public static final double kShootRPM = 3000; // preset shot, right trigger
    public static final double kTestDutyCycle = 0.2; // open-loop test, right bumper

    // Rough flywheel moment of inertia for simulation only (kg*m^2)
    public static final double kFlywheelMOI = 0.004;

    // VelocityVoltage gains, in volts per flywheel rotation per second (starting points - tune on robot)
    public static final double kS = 0.15;
    public static final double kV = 0.18;
    public static final double kP = 0.1;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kFlywheelToleranceRPM = 50;

    // Set to Opposed if a follower motor faces the opposite direction of motor 1
    public static final MotorAlignmentValue kshooterMotor2Alignment = MotorAlignmentValue.Opposed;
    public static final MotorAlignmentValue kshooterMotor3Alignment = MotorAlignmentValue.Opposed;
  }
}
