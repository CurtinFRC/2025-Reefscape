package org.curtinfrc.frc2025.subsystems.ejector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Ejector extends SubsystemBase {
  private final EjectorIO io;
  private final EjectorIOInputsAutoLogged inputs = new EjectorIOInputsAutoLogged();

  public Ejector(EjectorIO io) {
    this.io = io;
  }

  public final Trigger backSensor = new Trigger(() -> inputs.backSensor);
  public final Trigger frontSensor = new Trigger(() -> inputs.frontSensor);

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Ejector", inputs);
  }

  public Command stop() {
    return runOnce(() -> io.setVoltage(0));
  }

  public Command eject(double rotationsPerSecond) {
    return run(() -> {
          io.setVelocity(rotationsPerSecond);
        })
        .withName("Eject");
  }

  public Command setVoltage(double volts) {
    return run(() -> io.setVoltage(volts));
  }
}
