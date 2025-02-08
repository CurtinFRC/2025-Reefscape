package org.curtinfrc.frc2025;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.RobotBase;
import org.curtinfrc.frc2025.subsystems.drive.DriveConstants.DriveSetpoints;
import org.curtinfrc.frc2025.subsystems.elevator.ElevatorConstants.ElevatorSetpoints;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final RobotType robotType = RobotType.COMPBOT;
  public static final double ROBOT_X = 660; // mm
  public static final double ROBOT_Y = 680;
  public static final boolean devmode = true;

  public static final Mode getMode() {
    return switch (robotType) {
      case DEVBOT, COMPBOT -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case SIMBOT -> Mode.SIM;
    };
  }

  public static enum RobotType {
    /** Running in simulation */
    SIMBOT,

    /** Running competition robot. */
    COMPBOT,

    /** Running developer robot. */
    DEVBOT,
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static void main(String... args) {
    if (robotType == RobotType.SIMBOT) {
      System.out.println("Invalid robot type selected for deploy: SIMBOT.");
      System.exit(1);
    }
  }

  // Setpoints
  public static record Setpoint(ElevatorSetpoints elevatorSetpoint, DriveSetpoints driveSetpoint)
      implements StructSerializable {
    public static final Struct<Setpoint> struct = StructGenerator.genRecord(Setpoint.class);
  }
}
