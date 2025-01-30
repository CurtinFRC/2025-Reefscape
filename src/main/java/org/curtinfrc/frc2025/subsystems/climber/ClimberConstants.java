package org.curtinfrc.frc2025.subsystems.climber;

public class ClimberConstants {
  public static int grabberMotorPort = 99;

  public static final double grabberTargetVoltage = 4.0;

  private static double grabberGearRatio = 1.0; // 1:1
  private static double grabberTargetPositionRotations =
      0.0625; // 1/16 of a rotation / 22.5 degrees
  public static double grabberMotorTargetPositionRotations =
      grabberTargetPositionRotations * grabberGearRatio;

  public static double grabberkP = 1.0;
  public static double grabberkI = 0.0;
  public static double grabberkD = 0.0;
  public static double grabberkMinOutput = -1.0;
  public static double grabberkMaxOutput = 1.0;

  public static double grabberMotorkV = 1.0;
  public static double grabberMaxVelocity = 1.0;
  public static double grabberMaxAcceleration = 1.0;
  public static double grabberAllowedError = 0.0;

  public static double grabberTolerance = 0.1;
}
