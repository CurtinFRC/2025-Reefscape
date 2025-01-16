package org.curtinfrc.frc2025;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Set;
import org.curtinfrc.frc2025.Constants.Setpoints;
import org.curtinfrc.frc2025.subsystems.drive.Drive;
import org.curtinfrc.frc2025.subsystems.elevator.Elevator;

public class Superstructure {
  private final Elevator m_elevator;
  private final Drive m_drivebase;

  public Superstructure(Drive drive, Elevator elevator) {
    m_elevator = elevator;
    m_drivebase = drive;
  }

  public Command align(Setpoints setpoint) {
    return Commands.defer(
        () ->
            Commands.parallel(
                m_drivebase.autoAlign(setpoint.toPose(new Pose3d(m_drivebase.getPose()))),
                m_elevator.goToSetpoint(setpoint)),
        Set.of());
  }
}
