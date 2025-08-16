package org.curtinfrc.frc2025.subsystems.processor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

public class Processor extends SubsystemBase {
  private final ProcessorIO io;
  private final ProcessorIOInputsAutoLogged inputs = new ProcessorIOInputsAutoLogged();
  private final PIDController controller = new PIDController(11, 0, 0);

  public Processor(ProcessorIO io) {
    this.io = io;
  }

  public final Trigger processorSensor = new Trigger(() -> inputs.processorSensor);

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("processor", inputs);
  }

  public void goToPosition(double desiredPosition) {
    var currentPosition = inputs.armAbsolutePosition;
    var output = controller.calculate(currentPosition, desiredPosition);
    Logger.recordOutput("processor/desiredPosition", desiredPosition);
    io.armSetVoltage(output);
  }

  public void stopIntake() {
    io.intakeSetVoltage(0);
  }

  public void runIntake(double volts) {
    io.intakeSetVoltage(volts);
  }

  public Command stopArm() {
    return run(() -> io.armSetVoltage(0));
  }

  public void runArm(double volts) {
    io.armSetVoltage(volts);
  }

  public Command intake() {
    return run(() -> {
          goToPosition(0.15);
          runIntake(7);
        })
        .withName("Intake");
  }

  public Command outake() {
    return run(() -> {
          goToPosition(0.15);
          runIntake(-8);
        })
        .withName("Outake");
  }

  public Command idle() {
    return run(() -> {
          goToPosition(0);
          stopIntake();
        })
        .withName("Idle");
  }
}
