package org.curtinfrc.frc2025.subsystems.popper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Popper extends SubsystemBase {
  private final PopperIO io;
  private final PopperIOInputsAutoLogged inputs = new PopperIOInputsAutoLogged();

  public Popper(PopperIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Popper", inputs);
  }

  public Command setVoltage(double volts) {
    return run(() -> io.setVoltage(volts));
  }

  public Command stop() {
    return runOnce(() -> io.setVoltage(0));
  }
}
