package org.curtinfrc.frc2025;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.EnumMap;
import java.util.Map;
import org.curtinfrc.frc2025.Constants.Setpoint;
import org.curtinfrc.frc2025.subsystems.drive.Drive;
import org.curtinfrc.frc2025.subsystems.drive.DriveConstants.DriveSetpoints;
import org.curtinfrc.frc2025.subsystems.ejector.Ejector;
import org.curtinfrc.frc2025.subsystems.elevator.Elevator;
import org.curtinfrc.frc2025.subsystems.elevator.ElevatorConstants.ElevatorSetpoints;
import org.curtinfrc.frc2025.subsystems.intake.Intake;
import org.curtinfrc.frc2025.subsystems.popper.Popper;

public class Autos {
  private static class AlgaePoppedStates {
    enum AlgaeLocations {
      AB,
      CD,
      EF,
      GH,
      IJ,
      KL;

      public static AlgaeLocations from(DriveSetpoints point) {
        return switch (point) {
          case A, B -> AB;
          case C, D -> CD;
          case E, F -> EF;
          case G, H -> GH;
          case I, J -> IJ;
          case K, L -> KL;
          default -> throw new IllegalArgumentException("Unknown DriveSetpoint: " + point);
        };
      }
    }

    private final Map<AlgaeLocations, Boolean> locations = new EnumMap<>(AlgaeLocations.class);

    public AlgaePoppedStates() {
      for (AlgaeLocations loc : AlgaeLocations.values()) {
        locations.put(loc, false);
      }
    }

    public void pop(AlgaeLocations loc) {
      locations.put(loc, true);
    }

    public boolean isPopped(AlgaeLocations loc) {
      return locations.getOrDefault(loc, false);
    }
  }

  private static final AlgaePoppedStates state = new AlgaePoppedStates();

  private static class RepulsorAuto {
    private final Setpoint[] setpoints;
    private final DriveSetpoints hpSetpoint;
    private final Drive drive;
    private final Elevator elevator;
    private final Popper popper;
    private final Ejector ejector;
    private final Intake intake;

    public RepulsorAuto(
        Drive drive,
        Elevator elevator,
        Popper popper,
        Ejector ejector,
        Intake intake,
        DriveSetpoints hpSetpoint,
        Setpoint... reefSetpoints) {
      this.setpoints = reefSetpoints;
      this.drive = drive;
      this.hpSetpoint = hpSetpoint;
      this.elevator = elevator;
      this.popper = popper;
      this.ejector = ejector;
      this.intake = intake;
    }

    public Command build() {
      Command sequence = Commands.none();

      for (Setpoint setpoint : setpoints) {
        DriveSetpoints driveSetpoint = setpoint.driveSetpoint();
        AlgaePoppedStates.AlgaeLocations loc = AlgaePoppedStates.AlgaeLocations.from(driveSetpoint);

        sequence =
            sequence.andThen(
                drive
                    .autoAlign(driveSetpoint)
                    .until(drive.atSetpoint.and(ejector.backSensor.negate())));

        if (!state.isPopped(loc)) {
          sequence =
              sequence.andThen(
                  elevator
                      .goToSetpoint(ElevatorSetpoints.AlgaePop)
                      .until(elevator.atSetpoint)
                      .andThen(popper.setVoltage(5).withTimeout(2).andThen(popper.stop())));
          state.pop(loc);
        }

        sequence =
            sequence.andThen(
                elevator
                    .goToSetpoint(setpoint.elevatorSetpoint())
                    .until(ejector.backSensor.negate()));

        sequence =
            sequence.andThen(
                drive.autoAlign(hpSetpoint).until(drive.atSetpoint.and(intake.frontSensor)));
      }

      return sequence;
    }
  }

  private final Drive drive;
  private final Elevator elevator;
  private final Popper popper;
  private final Ejector ejector;
  private final Intake intake;

  public Autos(Drive drive, Elevator elevator, Popper popper, Ejector ejector, Intake intake) {
    this.drive = drive;
    this.elevator = elevator;
    this.popper = popper;
    this.ejector = ejector;
    this.intake = intake;
  }

  public Command basicAuto() {
    return new RepulsorAuto(
            drive,
            elevator,
            popper,
            ejector,
            intake,
            DriveSetpoints.LEFT_HP,
            new Setpoint(ElevatorSetpoints.L2, DriveSetpoints.A))
        .build();
  }
}
