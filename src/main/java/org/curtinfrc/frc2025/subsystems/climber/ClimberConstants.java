package org.curtinfrc.frc2025.subsystems.climber;

import org.curtinfrc.frc2025.util.LoggedTunableNumber;

public class ClimberConstants {
  // fully out: -8.67
  public static double targetPositionRotationsOut = -8.67;
  // fully in: 1.2
  public static double targetPositionRotationsIn = 1.2;
  public static double deployTolerance = 0.3;

  // motor stall thingy
  public static double stallingRPM = 15; // RPM
  public static double stallingCurrent = 60; // Amps

  public static final LoggedTunableNumber kP = new LoggedTunableNumber("Climber/kP", 0.1);
  public static final LoggedTunableNumber kI = new LoggedTunableNumber("Climber/kI", 0.0);
  public static final LoggedTunableNumber kD = new LoggedTunableNumber("Climber/kD", 0.0);

  public static double grabberTolerance = 0.2;
}
