package org.curtinfrc.frc2025;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Set;
import org.curtinfrc.frc2025.Constants.Setpoints;
import org.curtinfrc.frc2025.subsystems.drive.Drive;
import org.curtinfrc.frc2025.subsystems.elevator.Elevator;

public class Superstructure {
  private final Elevator elevator;
  private final Drive drive;

  public Superstructure(Drive drive, Elevator elevator) {
    this.elevator = elevator;
    this.drive = drive;
  }

  public Command align(Setpoints setpoint) {
    return Commands.defer(
        () ->
            Commands.parallel(
                // drive.autoAlign(setpoint.toPose(new Pose3d(drive.getPose()))),
                elevator.goToSetpoint(setpoint)),
        Set.of(elevator));
  }
}
