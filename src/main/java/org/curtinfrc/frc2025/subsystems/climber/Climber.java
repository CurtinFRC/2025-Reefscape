package org.curtinfrc.frc2025.subsystems.climber;

import static org.curtinfrc.frc2025.subsystems.climber.ClimberConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  public Climber(ClimberIO io) {
    this.io = io;
    setDefaultCommand(run(() -> stop()));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  public Command Raw() {
    return run(() -> {});
  }

  public Command goToSetpoint() {
    return run(() -> {});
  }

  public Command stop() {
    return run(() -> {});
  }

  public Command run() {
    return run(() -> {});
  }
}
