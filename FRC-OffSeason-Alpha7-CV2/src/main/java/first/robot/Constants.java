// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.wpilib.framework.RobotBase;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;
import org.wpilib.hardware.bus.CANPort;
import com.ctre.phoenix6.CANBus;
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

  public static class IntakeConstants {

    public static final CANPort kIntakeMotorCANPort = CANPort.CAN_S1;
    public static final CANBus kIntakeMotorCANBus = new CANBus(kIntakeMotorCANPort);

    // REV Through Bore (absolute, duty cycle) on the pivot shaft - reads pivot angle in degrees
    public static final int kIntakePivotEncoderID = 0;
    public static final double kIntakePivotEncoderOffset = 0; // raw encoder degrees when the pivot is stowed - measure on robot
    public static final boolean kIntakePivotEncoderInverted = false; // flip so the angle increases toward deploy

    public static final double kIntakePivotZero = 0;
    public static final double kIntakePivotDeployAngle = 104.14;
    public static final double kIntakePivotToleranceDegrees = 2.0; // how close counts as "there" for deploy/stow

    public static final int kIntakePivotID = 0;
    public static final int kIntakeRoller1ID = 0;
    public static final int kIntakeRoller2ID = 0;

    public static final double kIntakePivotDeployPower = 0.5;
    public static final double kIntakePivotStowPower = -0.5;

    public static final double kIntakeRollerInPower = 0.5;
    public static final double kIntakeRollerOutPower = -0.5;

    // Set to Opposed if roller 2 faces the opposite direction of roller 1
    public static final MotorAlignmentValue kIntakeRoller2Alignment = MotorAlignmentValue.Aligned;

    public static final double kIntakePivotGearRatio = 1.0; // motor rotations per pivot rotation - set to real ratio

    // TalonFX Slot0 PositionVoltage gains, in volts per pivot rotation of error (starting points - tune on robot)
    public static final double kIntakePIDp = 24.0;
    public static final double kIntakePIDi = 0.0;
    public static final double kIntakePIDd = 0.0;
  }

  private Constants() {}
}
