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

  // TODO: MAKE SETPOINTS
  public enum Setpoints {
    /* in mm */
    NONE(-1),
    L1(460),
    L2(810),
    L3(1210),
    COLLECT(950),
    STOW(0);

    public int setpoint;

    private Setpoints(int val) {
      this.setpoint = val;
    }
  };
}
