package org.curtinfrc.frc2025.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.common.hardware.VisionLEDMode;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
    public long[] allowedTags = new long[0];
  }

  /** Represents a robot pose sample used for pose estimation. */
  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      PoseObservationType type) {}

  public static enum PoseObservationType {
    MEGATAG_1,
    MEGATAG_2,
    PHOTONVISION,
    QUEST
  }

  public default void updateInputs(VisionIOInputs inputs) {}

  public default void allowTags(long[] ids) {}

  public default void setLEDMode(VisionLEDMode mode) {}
}
