package org.curtinfrc.frc2025.subsystems.climber;

public class ClimberConstants {
  public static int grabberMotorPort = 9;

  public static double targetPositionRotations = 0.0625; // 1/16 of a rotation / 22.5 degrees

  public static int currentLimit = 60;

  public static double kP = 1.0;
  public static double kI = 1.0;
  public static double kD = 0.2;

  public static double grabberTolerance = 0.1;
}
