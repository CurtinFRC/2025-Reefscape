package org.curtinfrc.frc2025.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
    // setDefaultCommand(run(() -> io.setIntakeVolts(0)));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public void setIntakeVolts(double volts) {
    io.setIntakeVolts(volts);
  }

  public Command stop() {
    return run(() -> io.setIntakeVolts(0));
  }

  public Command intakeCommand() {
    return run(() -> io.setIntakeVolts(6));
  }

  public Command goToTargetRPM() {
    return run(() -> io.achieveRPM()).until(() -> io.intakeAtRPM());
  }
}
