package org.curtinfrc.frc2025.subsystems.climber;

public class ClimberConstants {
  public static int pivotMotorPort = 99;
  public static int grabberMotorPort = 99;

  public static final double pivotTargetVoltage = 4.0; // find actual values
  public static final double grabberTargetVoltage = 4.0; // "               "

  private static double pivotGearRatio = 160.0;
  private static double pivotTargetPositionRotations = 0.0125 * Math.PI; // 45 degrees
  public static double pivotMotorTargetPositionRotations =
      pivotTargetPositionRotations * pivotGearRatio;

  public static double pivotkP = 1.0;
  public static double pivotkI = 0.0;
  public static double pivotkD = 0.0;
  public static double pivotkMinOutput = -1.0;
  public static double pivotkMaxOutput = 1.0;

  public static double pivotkV = 1.0;
  public static double pivotMaxVelocity = 1.0;
  public static double pivotMaxAcceleration = 1.0;
  public static double pivotAllowedError = 0.0;

  private static double grabberGearRatio = 100.0;
  private static double grabberTargetPositionRotations = 0.25 * Math.PI; // 90 degrees
  public static double grabberMotorTargetPositionRotations =
      grabberTargetPositionRotations * grabberGearRatio;

  public static double grabberkP = 1.0;
  public static double grabberkI = 0.0;
  public static double grabberkD = 0.0;
  public static double grabberkMinOutput = -1.0;
  public static double grabberkMaxOutput = 1.0;

  public static double grabberkV = 1.0;
  public static double grabberMaxVelocity = 1.0;
  public static double grabberMaxAcceleration = 1.0;
  public static double grabberAllowedError = 0.0;
}
