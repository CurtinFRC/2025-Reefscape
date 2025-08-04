package org.curtinfrc.frc2025.subsystems.elevator;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator;
import edu.wpi.first.util.struct.StructSerializable;
import org.curtinfrc.frc2025.util.LoggedTunableNumber;

public class ElevatorConstants {
  // TODO: TUNE PID and ff
  public static double tolerance = 0.021;
  public static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", 20.0);
  public static final LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI", 0.0);
  public static final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", 0.0);
  public static final LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV", 0.0);
  public static final LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA", 0.01);
  public static final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG", 0.0);

  public static final LoggedTunableNumber climbkP =
      new LoggedTunableNumber("Elevator/climbkP", 45.0);
  public static final LoggedTunableNumber climbkI =
      new LoggedTunableNumber("Elevator/climbkI", 0.0);
  public static final LoggedTunableNumber climbkD =
      new LoggedTunableNumber("Elevator/climbkD", 0.0);

  public static enum ElevatorSetpoints implements StructSerializable {
    L1(0.03),
    L2(0.221),
    AlgaePopLow(0),
    L3(0.611),
    AlgaePopHigh(0.38),
    BASE(0.01),
    climbPrep(0.3),
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
