package org.curtinfrc.frc2025.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "front-left";
  public static String camera1Name = "front-right";
  public static String camera2Name = "back-left";
  public static String camera3Name = "back-right";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCamera0 =
      new Transform3d(0.1655, 0.2008, 0.299, Rotation3d.kZero);
  public static Transform3d robotToCamera1 =
      new Transform3d(0.1655, -0.2008, 0.299, Rotation3d.kZero);
  public static Transform3d robotToCamera2 =
      new Transform3d(0.145943, 0.1723, 0.890575, new Rotation3d(0, -Math.PI / 4, Math.PI));
  public static Transform3d robotToCamera3 =
      new Transform3d(0.145943, -0.1723, 0.890575, new Rotation3d(0, -Math.PI / 4, Math.PI));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.07; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        0.8, // Camera 1
        0.8, // Camera 2
        1.0 // Camera 3
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.2; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
  public static final Translation2d questResetPosition = Translation2d.kZero;
}
