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
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  public Command stop() {
    return run(() -> io.setVoltage(0.0));
  }

  public Command home(double voltage) {
    return run(() -> io.setVoltage(voltage)).until(this::stalled);
  }

  public boolean stalled() {
    return inputs.currentAmps > 2 && inputs.angularVelocityRotationsPerMinute > 10;
  }
}
