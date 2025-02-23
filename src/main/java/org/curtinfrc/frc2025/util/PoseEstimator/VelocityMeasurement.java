package org.curtinfrc.frc2025.util.PoseEstimator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class VelocityMeasurement extends Measurement {

  public VelocityMeasurement(
      ChassisSpeeds measurement, Double stdDev, Double angularStdDev, Double newTime) {
    pose = new Pose2dVector();
    acceleration = new Pose2dVector();

    velocity =
        new Pose2dVector(
            measurement.vxMetersPerSecond,
            measurement.vyMetersPerSecond,
            Rotation2d.fromRadians(measurement.omegaRadiansPerSecond),
            stdDev,
            angularStdDev);

    time = newTime;
  }
}
