package org.curtinfrc.frc2025.subsystems.intake;

public class IntakeConstants {
  public static double intakeVolts = 6;
  public static int intakeCurrentLimit = 40;
  public static double motorReduction = 1.0;
  public static double intakePort = 99;
  public static double intakeMaxVelocity = 550;
  public static double intakeTolerance = 0.01;
  // public static double goalRPM = 4000;
  public static double goalRPM = 500;
  public static double intakeLoopError = 0.1;

  public static double kP = 1;
  public static double kI = 0;
  public static double kD = 0;
  public static double kV = 473;

  public static double intakeID = 99;

  public enum intakeSetPoints {}
}
