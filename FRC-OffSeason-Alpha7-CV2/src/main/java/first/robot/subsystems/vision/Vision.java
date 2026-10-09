package first.robot.subsystems.vision;

import static first.robot.Constants.LimelightConstants.*;

import first.robot.Constants;
import first.robot.Constants.Mode;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

public class Vision extends SubsystemBase {
  @FunctionalInterface
  public interface VisionConsumer {
    void accept(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs);
  }

  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
  private final VisionConsumer consumer;
  private final Supplier<Rotation2d> heading;
  private final DoubleSupplier yawRateRadPerSec;
  private final Alert disconnectedAlert =
      new Alert("limelightDisconnected", "Limelight disconnected.", Level.MEDIUM);

  public Vision(VisionIO io, VisionConsumer consumer,
      Supplier<Rotation2d> heading, DoubleSupplier yawRateRadPerSec) {
    this.io = io;
    this.consumer = consumer;
    this.heading = heading;
    this.yawRateRadPerSec = yawRateRadPerSec;
  }

  @Override
  public void periodic() {
    io.setRobotYaw(heading.get());
    io.updateInputs(inputs);
    Logger.processInputs("Vision", inputs);
    disconnectedAlert.set(!inputs.connected && Constants.getMode() == Mode.REAL);

    boolean spinning = Math.abs(yawRateRadPerSec.getAsDouble()) > kMAX_YAW_RATE_RAD_PER_SEC;
    List<Pose2d> accepted = new ArrayList<>();
    for (var obs : inputs.poseObservations) {
      if (spinning) continue;
      consumer.accept(obs.pose(), obs.timestamp(),
          VecBuilder.fill(obs.xyStdDev(), obs.xyStdDev(), kTHETA_STD_DEV_UNTRUSTED));
      accepted.add(obs.pose());
    }
    Logger.recordOutput("Vision/AcceptedPoses", accepted.toArray(new Pose2d[0]));
    Logger.recordOutput("Vision/RejectedForYawRate", spinning && inputs.poseObservations.length > 0);
  }
}