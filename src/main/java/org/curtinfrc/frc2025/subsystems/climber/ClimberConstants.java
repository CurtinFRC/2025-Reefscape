package org.curtinfrc.frc2025.subsystems.climber;

public class ClimberConstants {
  public static int grabberMotorPort = 9;

  public static double targetPositionRotationsOut = -8.67;
  public static double targetPositionRotationsIn = 1.2;
  public static double deployTolerance = 0.3;
  // fully in: 1.3
  // fully out: -8.67

  // motor stall thingy
  public static double stallingRPM = 15; // RPM
  public static double stallingCurrent = 60; // Amps

  public static int currentLimit = 60;

  public static double kP = 0.1;
  public static double kI = 0.0;
  public static double kD = 0.0;

  public static double grabberTolerance = 0.2;
}
