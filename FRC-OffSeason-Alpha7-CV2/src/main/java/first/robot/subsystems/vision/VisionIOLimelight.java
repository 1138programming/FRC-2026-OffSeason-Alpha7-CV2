package first.robot.subsystems.vision;

import com.limelightvision.IMUMode;
import com.limelightvision.Limelight;
import com.limelightvision.PoseEstimate;
import com.limelightvision.PoseEstimateConfig;
import com.limelightvision.PoseEstimateType;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;

public class VisionIOLimelight implements VisionIO {
  private final Limelight camera;

  public VisionIOLimelight(String name, Pose3d robotToCamera) {
    camera = new Limelight(name, robotToCamera)
        .withPoseEstimateConfig_MT2(PoseEstimateConfig.defaultMT2());
    camera.setIMUMode(IMUMode.EXTERNAL); 
  }

  @Override
  public void setRobotYaw(Rotation2d yaw) {
    Limelight.setSharedRobotOrientation(yaw.getDegrees());
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    PoseEstimate[] estimates = camera.readAcceptedPoseEstimates(PoseEstimateType.MT2_WPIBLUE);
    inputs.connected = camera.isConnected();
    inputs.poseObservations = new PoseObservation[estimates.length];
    for (int i = 0; i < estimates.length; i++) {
      PoseEstimate e = estimates[i];
      inputs.poseObservations[i] = new PoseObservation(
          e.timestampSeconds, e.pose, e.stdDevs.get(0, 0), e.fieldedTagCount, e.avgTagDistanceMeters);
    }
  }
}