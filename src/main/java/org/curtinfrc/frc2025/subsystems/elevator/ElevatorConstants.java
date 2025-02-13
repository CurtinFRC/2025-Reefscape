package org.curtinfrc.frc2025.subsystems.elevator;

public class ElevatorConstants {
  public static int resetPort = 99;
  public static int motorPort = 31;

  // TODO: TUNE PID and ff
  public static double tolerance = 1;
  public static double kP = 0.5;
  public static double kD = 0;
  public static double kV = 0;
  public static double kA = 0.01;
  public static double kG = 0;

  public static final double pulleyRadiusMeters = 0.002927789;
  public static final double maxHeightMeters = 0.8;
  public static final double carriageToGround = 0.06229734;

  public static double positionRotationsToMetres(double rotations) {
    return rotations * Math.PI * 2 * pulleyRadiusMeters;
  }

  public static double positionMetresToRotations(double metres) {
    return metres / (Math.PI * 2 * pulleyRadiusMeters);
  }
}
