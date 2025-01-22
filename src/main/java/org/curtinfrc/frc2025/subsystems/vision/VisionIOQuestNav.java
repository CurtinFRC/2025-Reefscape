package org.curtinfrc.frc2025.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.curtinfrc.frc2025.util.QuestNav;
import org.littletonrobotics.junction.Logger;

public class VisionIOQuestNav implements VisionIO {
  @Override
  // we skip updating target info cause we don't have it
  public void updateInputs(VisionIOInputs inputs) {
    // Update connection status based on whether an update has been seen in the last 250ms
    inputs.connected = QuestNav.instance.connected();
    Logger.recordOutput("QuestResetPose", QuestNav.instance.resetPosition);

    inputs.poseObservations =
        new PoseObservation[] {
          new PoseObservation(
              QuestNav.instance.timestamp(),
              new Pose3d(QuestNav.instance.getPose()),
              0.0,
              1,
              1.5,
              PoseObservationType.QUEST)
        };

    QuestNav.instance.cleanUpQuestNavMessages();
  }
}
