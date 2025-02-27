package org.curtinfrc.frc2025.subsystems.elevator;

public class ElevatorConstants {
  public static int resetPort = 0;
  public static int leaderPort = 31;
  public static int followerPort = 32;
  public static final int currentLimit = 60;

  public static double tolerance = 0.01;
  public static double maxVelocity = 3.50;
  public static double maxAccel = 51.57;

  public static double kP = 0.0001;
  public static double kD = 0;
  public static double kS = 0.1;
  public static double kV = 3.31;
  public static double kA = 0.05;
  public static double kG = 0.2;

  public static final double pulleyRadiusMeters = 0.03055;
  public static final double gearing = 8.1818;
  public static final double maxHeightMeters = 0.8;
  public static final double carriageToGround = 0.06229734;

  public static double positionRotationsToMetres(double rotations) {
    return rotations * Math.PI * 2 * pulleyRadiusMeters / gearing;
  }

  public static double positionMetresToRotations(double metres) {
    return metres / (Math.PI * 2 * pulleyRadiusMeters) * gearing;
  }
}
