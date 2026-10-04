package first.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public PoseObservation[] poseObservations = new PoseObservation[0];
  }

  /** One MegaTag2 estimate that already passed LimelightLib's filters. */
  public static record PoseObservation(
      double timestamp, Pose2d pose, double xyStdDev, int tagCount, double avgTagDistance) {}

  public default void setRobotYaw(Rotation2d yaw) {}

  public default void updateInputs(VisionIOInputs inputs) {}
}