package org.curtinfrc.frc2025;

import org.curtinfrc.frc2025.Constants.Setpoints;
import org.curtinfrc.frc2025.subsystems.drive.Drive;
import org.curtinfrc.frc2025.subsystems.elevator.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Superstructure {
    private final Elevator m_elevator;
    private final Drive m_drivebase;

    public Superstructure(Drive drive, Elevator elevator) {
        m_elevator = elevator;
        m_drivebase = drive;
    }

    public Command align(Setpoints setpoint) {
        return Commands.parallel(m_drivebase.autoAlign(setpoint), m_elevator.goToSetpoint(setpoint));
    }
}
