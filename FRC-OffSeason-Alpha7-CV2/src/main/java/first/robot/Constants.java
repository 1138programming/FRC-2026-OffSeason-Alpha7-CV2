// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import org.wpilib.framework.RobotBase;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

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

  // ------- Subsystem constants ----- //
  public static class ActiveFloorConstants {
    public static final int kFLOOR_ROLLER_MOTOR_ID = 9;

    public static final double kFLOOR_POWER_INWARD = 0.5;
    public static final double kFLOOR_POWER_OUTWARD = -0.5;
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
  public static class IndexerConstants{
    public static final CANPort kS0 = CANPort.CAN_S0;
    public static final int kINDEXER_LEFT_ID = 0;
    public static final int kINDEXER_RIGHT_ID = 0;

    public static final double kINDEX_IN_POWER = 0.576;
    public static final double kINDEX_OUT_POWER = -0.576;
  }

  private Constants() {}

  public static class ShooterConstants{
    public static final CANPort kshooterMotorCANbus = CANPort.CAN_S0;

    public static final int kshooterMotor1ID = 0;
    public static final int kshooterMotor2ID = 0;
    public static final int kshooterMotor3ID = 0;

    public static final double kFlywheelGearRatio = 2.0 / 3.0; // flywheel rotations per motor rotation

    // VelocityVoltage gains, in volts per flywheel rotation per second (starting points - tune on robot)
    public static final double kS = 0.15;
    public static final double kV = 0.18;
    public static final double kP = 0.1;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kFlywheelToleranceRPM = 50;

    // Set to Opposed if a follower motor faces the opposite direction of motor 1
    public static final MotorAlignmentValue kshooterMotor2Alignment = MotorAlignmentValue.Aligned;
    public static final MotorAlignmentValue kshooterMotor3Alignment = MotorAlignmentValue.Aligned;
  }
}
