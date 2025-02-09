package org.curtinfrc.frc2025.util;

import edu.wpi.first.math.geometry.Pose3d;
import org.curtinfrc.frc2025.Constants;

public final class PoseUtil {
  public static Pose3d mapPose(Pose3d pose) {
    double angle = pose.getRotation().getAngle();
    return new Pose3d(
        pose.getX() + Math.cos(angle) * Constants.ROBOT_X / 2000.0,
        pose.getY() + Math.sin(angle) * Constants.ROBOT_Y / 2000.0,
        0.0,
        pose.getRotation());
  }
}
