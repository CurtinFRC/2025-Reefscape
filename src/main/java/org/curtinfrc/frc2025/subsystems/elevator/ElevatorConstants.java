package org.curtinfrc.frc2025.subsystems.elevator;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator;
import edu.wpi.first.util.struct.StructSerializable;

public class ElevatorConstants {
  public static int resetPort = 0;
  public static int leaderPort = 31;
  public static int followerPort = 32;
  public static final int currentLimit = 60;

  // TODO: TUNE PID and ff
  public static double tolerance = 0.021;
  public static double kP = 25; // 50
  public static double kD = 0;
  public static double kV = 0;
  public static double kA = 0.01;
  public static double kG = 0;

  public static double climbkP = 45;
  public static double climbkI = 0;
  public static double climbkD = 0;

  public static final double pulleyRadiusMeters = 0.03055;
  public static final double gearing = 8.1818;
  public static final double maxHeightMeters = 0.8;
  public static final double carriageToGround = 0.06229734;

  public static double positionRotationsToMetres(double rotations) {
    return rotations * Math.PI * 2 * pulleyRadiusMeters / gearing;
  }

  public static double positionMetresToRotations(double metres) {
    return metres / (Math.PI * 2 * pulleyRadiusMeters) * gearing;
  }

  // TODO
  public static enum ElevatorSetpoints implements StructSerializable {
    L1(0),
    L2(0.2),
    AlgaePopLow(0.1),
    L3(0.59735),
    AlgaePopHigh(0.55),
    BASE(0.01),
    climbPrep(0.37),
    climbAttempt(0.5),
    climbed(0.01);

    public final double setpoint;

    ElevatorSetpoints(double setpoint) {
      this.setpoint = setpoint;
    }

    public static ElevatorSetpoints getPopPoint(ElevatorSetpoints point) {
      switch (point) {
        case L2:
          return AlgaePopLow;
        case L3:
          return AlgaePopHigh;
        default:
          return BASE;
      }
    }

    public static final Struct<ElevatorSetpoints> struct =
        StructGenerator.genEnum(ElevatorSetpoints.class);
  }
}
