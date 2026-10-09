package first.robot;

import static first.robot.Constants.FieldConstants.*;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation2d;

class ShotTargetTest {
  @Test
  void aimsAtOwnHubFromAllianceZone() {
    assertEquals(kBLUE_HUB, RobotContainer.shotTarget(new Translation2d(2.0, 4.0), false));
    assertEquals(kRED_HUB, RobotContainer.shotTarget(new Translation2d(14.5, 4.0), true));
  }

  @Test
  void shuttlesToOwnAllianceZoneFromNeutralZone() {
    Translation2d midfield = new Translation2d(kFIELD_LENGTH / 2, 2.0);
    assertEquals(kBLUE_SHUTTLE_TARGET, RobotContainer.shotTarget(midfield, false));
    assertEquals(kRED_SHUTTLE_TARGET, RobotContainer.shotTarget(midfield, true));
  }
}
