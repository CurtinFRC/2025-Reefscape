package org.curtinfrc.frc2025.subsystems.elevator;

public class ElevatorConstants {
  public static int resetPort = 98;
  public static int motorPort = 31;
  public static double radiusMeters = 0.012865;
  public static double massKgs = 1; // TODO

  // TODO: TUNE PID and ff
  public static double tolerance = 0.1;
  public static double kP = 0.5;
  public static double kD = 0;
  public static double kV = 0;
  public static double kG = 0;

  public static double heightMetersToRotations(double heightMeters) {
    return heightMeters * 2 * Math.PI * radiusMeters;
  }

  public static double rotationsToHeightMeters(double rotations) {
    return rotations / (2 * Math.PI * radiusMeters);
  }
}
