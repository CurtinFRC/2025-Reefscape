package org.curtinfrc.subsystems.ejector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Ejector extends SubsystemBase {
  private final EjectorIO io;
  private final EjectorIOInputsAutoLogged inputs = new EjectorIOInputsAutoLogged();

  public Ejector(EjectorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Ejector", inputs);
  }

  public void setEjectorVolts(double volts) {
    io.setEjectorVolts(volts);
  }

  public Command stop() {
    return run(() -> io.setEjectorVolts(0));
  }

  public Command ejectCommand() {
    return run(() -> io.setEjectorVolts(6));
  }

  public Command goToEjectorTargetRPM() {
    return run(() -> io.achieveEjectorRPM()).until(() -> io.ejectorAtRPM());
  }
}
