package org.curtinfrc.frc2025.util;

import org.curtinfrc.frc2025.Constants;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public final class PoseUtil {
    public static Pose3d mapPose(Pose2d pose) {
    double angle = pose.getRotation().getRadians();
    double cos = Math.cos(angle);
    double sin = Math.sin(angle);
    double dimx = Constants.DIMENSIONS_X / 2000;
    double dimy = Constants.DIMENSIONS_Y / 2000;
    double xdiff = cos * dimx;
    double ydiff = sin * dimy;
    double forwardX = pose.getX() + xdiff;
    double forwardY = pose.getY() + ydiff;

    Logger.recordOutput("PoseMapping/angle", angle);
    Logger.recordOutput("PoseMapping/forwardX", xdiff);
    Logger.recordOutput("PoseMapping/forwardY", ydiff);

    Logger.recordOutput("PoseMapping/cos", cos);
    Logger.recordOutput("PoseMapping/sin", sin);
    Logger.recordOutput("PoseMapping/dimx", dimx);
    Logger.recordOutput("PoseMapping/dimy", dimy);

    Pose3d new_pose = new Pose3d(forwardX, forwardY, 0, new Rotation3d(0, 0, angle));
    
    Logger.recordOutput("PoseMapping/final", new_pose);
    return new_pose;
  }

}
