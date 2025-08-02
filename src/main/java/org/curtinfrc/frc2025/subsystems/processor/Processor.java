package org.curtinfrc.frc2025.subsystems.processor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.PIDController;

public class Processor extends SubsystemBase {
  private final ProcessorIO io;
  private final ProcessorIOInputsAutoLogged inputs = new ProcessorIOInputsAutoLogged();
  private final PIDController controller = new PIDController(0, 0, 0);

  public Processor(ProcessorIO io) {
    this.io = io;
  }

  public final Trigger processorSensor = new Trigger(() -> inputs.processorSensor);

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("processor", inputs);
  }

  public Command goToPosition(double desiredPosition) {
    return run(() -> {
      var currentPosition = inputs.armAbsolutePosition;
      var output = controller.calculate(currentPosition, desiredPosition);
      Logger.recordOutput("processor/desiredPosition", desiredPosition);
      io.armSetVoltage(output);
    });
  }

  public Command stopIntake() {
    return run(() -> io.intakeSetVoltage(0));
  }

  public Command runIntake(double volts) {
    return run(() -> io.intakeSetVoltage(volts));
  }

  public Command runIntake() {
    return run(() -> io.intakeSetVoltage(3)); // change voltage
  }

  public Command stopArm() {
    return run(() -> io.armSetVoltage(0));
  }

  public Command runArm(double volts) {
    return run(() -> io.armSetVoltage(volts));
  }

  public Command runArm() {
    return run(() -> io.armSetVoltage(3)); // change voltage
  }
}
