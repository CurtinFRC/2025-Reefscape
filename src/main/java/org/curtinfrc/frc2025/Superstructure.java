package org.curtinfrc.frc2025;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.Supplier;

import org.curtinfrc.frc2025.Constants.Setpoint;
import org.curtinfrc.frc2025.subsystems.drive.Drive;
import org.curtinfrc.frc2025.subsystems.drive.DriveConstants.DriveSetpoints;
import org.curtinfrc.frc2025.subsystems.ejector.Ejector;
import org.curtinfrc.frc2025.subsystems.elevator.Elevator;
import org.curtinfrc.frc2025.subsystems.intake.Intake;

public class Superstructure {
  private final Intake intake;
  private final Ejector ejector;
  private final Elevator elevator;
  private final Drive drive;

  public final Trigger hasGamepiece;
  public final Trigger storingGamepiece;
  public final Trigger nearScoringPosition;
  public final Trigger scoring;
  public final Trigger noGamepiece;

  public Superstructure(Intake intake, Ejector ejector, Elevator elevator, Drive drive) {
    this.intake = intake;
    this.ejector = ejector;
    this.elevator = elevator;
    this.drive = drive;

    hasGamepiece =
        intake.frontSensor.or(intake.backSensor).or(ejector.frontSensor).or(ejector.backSensor);
    storingGamepiece =
        intake
            .frontSensor
            .negate()
            .and(intake.backSensor.negate())
            .and(ejector.frontSensor)
            .and(ejector.backSensor);
    nearScoringPosition = drive.almostAtSetpoint; // TODO verify is reef setpoint
    scoring = drive.atSetpoint.and(ejector.backSensor); // TODO verify is reef setpoint
    noGamepiece =
        intake
            .frontSensor
            .negate()
            .and(intake.backSensor.negate())
            .and(ejector.frontSensor.negate())
            .and(ejector.backSensor.negate());
  }

  public Command autoScore(Supplier<Setpoint> scoringSetpoint) {
    return Commands.idle();
  }

  public Command intake(Supplier<Setpoint> intakingSetpoint) {
    return Commands.idle();
  }
}
