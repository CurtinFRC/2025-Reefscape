package org.curtinfrc.frc2025.subsystems.elevator;

public class ElevatorConstants {
  public static int resetPort = 99;
  public static int motorPort = 31;

  // TODO: TUNE PID and ff
  public static double tolerance = 1;
  public static double kP = 0.5;
  public static double kD = 0;
  public static double kV = 0;
  public static double kG = 0;

  public static final double pulleyRadiusMeters = 0.02927789; // TODO get this
  public static final double maxHeightMeters = 1.441;
  public static final double carriageToGround = 0.06229734;
}
