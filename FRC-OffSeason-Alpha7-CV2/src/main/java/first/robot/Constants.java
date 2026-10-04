// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.wpilib.framework.RobotBase;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.Units;
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

  public static class LimelightConstants{
    public static final String klimelightName = "limelight";

  public static final Pose3d kROBOT_TO_CAMERA = new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));
  public static final double kMAX_YAW_RATE_RAD_PER_SEC = Units.degreesToRadians(360);
  public static final double kTHETA_STD_DEV_UNTRUSTED = 9999999; // gyro owns heading
  }

  /** REBUILT (2026) welded field, WPILib blue-origin meters. HUB centers derived from face tags. */
  public static class FieldConstants {
    public static final double kFIELD_LENGTH = 16.541;
    public static final double kFIELD_WIDTH = 8.069;
    public static final Translation2d kBLUE_HUB = new Translation2d(4.6255, 4.0346);
    public static final Translation2d kRED_HUB = new Translation2d(11.9155, 4.0346);
  }

  private Constants() {}
}
