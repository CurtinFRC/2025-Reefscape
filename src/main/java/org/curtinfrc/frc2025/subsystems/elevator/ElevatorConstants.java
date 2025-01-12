package org.curtinfrc.frc2025.subsystems.elevator;

public class ElevatorConstants {
  public static int motorCount = 2;
  public static int distanceSensorPort = 99;
  public static int motorPort = 99;

  public static double tolerance = 0.01;

  public static double maxVel = 5676;
  public static double maxAccel = 0;
  public static double allowedErr = 0;

  // TODO: TUNE PID
  public static double kP = 1;
  public static double kI = 0;
  public static double kD = 0;
  public static double kMinOutput = 0;
  public static double kMaxOutput = 0;
  public static double kV = 473;
}
