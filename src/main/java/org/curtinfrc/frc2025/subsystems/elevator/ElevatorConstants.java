package org.curtinfrc.frc2025.subsystems.elevator;

public class ElevatorConstants {
  // reduction for elevator neo is 10.7 : 1
  // TODO: GET VALUES FOR SOME OF THESE
  public static int motorCount = 1; // Number of motors in the elevator system
  public static int distanceSensorPort = 0; // Port for distance sensor (if used)
  public static int motorPort = 1; // Motor controller port
  public static double tolerance = 0.01; // Velocity tolerance (rotations per second)
  public static double positionTolerance = 0.002; // Position tolerance (rotations)
  public static double friction = 0.05; // Estimated system friction (Nm) TODO
  public static double stableVelocityThreshold =
      0.005; // Threshold for velocity to be considered stable

  // Motion profile constraints
  public static double maxVel = 5676 / 10.7; // Max velocity in RPM
  public static double maxAccel = 38197; // Max acceleration in RPM per second
  public static double allowedErr = 0.002; // Allowable position error for stability

  // Model Predictive Control Gains
  public static double kP = 6; // Proportional gain TODO
  public static double kI = 0.01; // Integral gain TODO
  public static double kD = 0.1; // Derivative gain TODO
  public static double kA = 0.3;
  public static double kMinOutput = -12.0; // Minimum voltage output
  public static double kMaxOutput = 12.0; // Maximum voltage output
  public static double kV = 473; // Voltage constant (i for it from the datasheet)
}
