package org.curtinfrc.frc2025;

import java.lang.reflect.Array;
import java.util.List;

import org.curtinfrc.frc2025.subsystems.drive.Drive;
import org.curtinfrc.frc2025.subsystems.drive.DriveConstants.DriveSetpoints;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Autos {
  private static class RepulsorAuto {
    private final DriveSetpoints[] setpoints;
    private final DriveSetpoints hpSetpoint;
    private final Drive drive;
    private int next_idx = 0;

    public RepulsorAuto(Drive d, DriveSetpoints hpStation, DriveSetpoints... reefSetpoints) {
        this.setpoints = reefSetpoints;
        this.drive = d;
        this.hpSetpoint = hpStation;
    }

    public Command next() {
        if (next_idx >= setpoints.length) return Commands.none(); 
        return Commands.run(() -> drive.autoAlign(setpoints[next_idx++]), drive);
    }

    public Command get() {
        Command sequence = Commands.sequence();
        for (DriveSetpoints setpoint : setpoints) {
            sequence = sequence.andThen(next());
            sequence = sequence.andThen(Commands.run(() -> drive.autoAlign(hpSetpoint), drive));
        }
        return sequence;
    }
  }

  private final Drive drive;

  public Autos(Drive drive) {
    this.drive = drive;
  }

  public Command basicAuto() {
    return new RepulsorAuto(drive, DriveSetpoints.LEFT_HP, DriveSetpoints.A).get();
  }
}
