package org.curtinfrc.frc2025.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public final Trigger frontSensor = new Trigger(() -> inputs.frontSensor);
  public final Trigger backSensor = new Trigger(() -> inputs.backSensor);

  @AutoLogOutput(key = "Intake/motorStalled")
  public final Trigger motorStalled =
      new Trigger(
          () ->
              inputs.angularVelocityRotationsPerMinute < 6
                  && inputs.appliedVolts > 0
                  && inputs.currentAmps > 30);

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public Command stop() {
    return run(() -> io.setVoltage(0));
  }

  public Command intake(double volts) {
    return run(() -> io.setVoltage(volts));
  }

  public Command intake() {
    return run(() -> io.setVoltage(3));
  }
}
