package org.curtinfrc.frc2025.subsystems.climber;

public class ClimberConstants {
  public static int grabberMotorPort = 99;

  public static final double grabberTargetVoltage = 4.0;

  public static double grabberTimeout = 5.0;

  public static double targetPositionRotations = 0.0625; // 1/16 of a rotation / 22.5 degrees

  public static int currentLimit = 60; // TODO: find actual value

  public static double kP = 1;
  public static double kI = 1.0;
  public static double kD = 0.2;
  public static double kMinOutput = -1.0;
  public static double kMaxOutput = 1.0;

  public static double grabberMotorkV = 1.0;
  public static double grabberMaxVelocity = 1.0;
  public static double grabberMaxAcceleration = 1.0;
  public static double grabberAllowedError = 0.0;

  public static double grabberTolerance = 0.1;
}
