package org.curtinfrc.frc2025.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj.RobotController;

public class VisionIOQuestNav implements VisionIO {
  // Configure Network Tables topics (questnav/...) to communicate with the Quest HMD
  NetworkTable nt4Table = NetworkTableInstance.getDefault().getTable("questnav");
  private IntegerSubscriber questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);
  private IntegerPublisher questMosi = nt4Table.getIntegerTopic("mosi").publish();
  private StructSubscriber<Pose2d> resetPose =
      NetworkTableInstance.getDefault()
          .getStructTopic("Auto Chooser/resetPose", Pose2d.struct)
          .subscribe(Pose2d.kZero);

  // Subscribe to the Network Tables oculus data topics
  private DoubleSubscriber questTimestamp = nt4Table.getDoubleTopic("timestamp").subscribe(0.0f);
  private FloatArraySubscriber questEulerAngles =
      nt4Table.getFloatArrayTopic("eulerAngles").subscribe(new float[] {0.0f, 0.0f, 0.0f});
  private FloatArraySubscriber questPosition =
      nt4Table.getFloatArrayTopic("position").subscribe(new float[] {0.0f, 0.0f, 0.0f});

  // Local heading helper variables
  private float yaw_offset = 0.0f;

  @Override
  // we skip updating target info cause we don't have it
  public void updateInputs(VisionIOInputs inputs) {
    // Update connection status based on whether an update has been seen in the last 250ms
    inputs.connected =
        ((RobotController.getFPGATime() - questTimestamp.getLastChange()) / 1000) < 250;

    inputs.poseObservations =
        new PoseObservation[] {
          new PoseObservation(
              questTimestamp.get(),
              new Pose3d(getOculusPose()),
              0.0,
              3,
              1.0,
              PoseObservationType.QUEST)
        };

    cleanUpQuestNavMessages();
  }

  // Get the yaw Euler angle of the headset
  private float getOculusYaw() {
    float[] eulerAngles = questEulerAngles.get();
    var ret = eulerAngles[1] - yaw_offset;
    ret %= 360;
    if (ret < 0) {
      ret += 360;
    }
    return -ret;
  }

  private Translation2d getOculusPosition() {
    float[] oculusPosition = questPosition.get();
    return new Translation2d(oculusPosition[2], -oculusPosition[0]);
  }

  private Pose2d getOculusPose() {
    var pose = resetPose.get();
    var oculousPositionCompensated = getOculusPosition().minus(pose.getTranslation()); // 6.5
    return new Pose2d(
        oculousPositionCompensated,
        Rotation2d.fromDegrees(getOculusYaw()).minus(pose.getRotation()));
  }

  // Clean up questnav subroutine messages after processing on the headset
  private void cleanUpQuestNavMessages() {
    if (questMiso.get() == 99) {
      questMosi.set(0);
    }
  }
}
