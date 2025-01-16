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

  public Command grabberRaw() {
    return run(() -> io.setGrabberVoltage(grabberTargetVoltage));
  }

  public Command pivotRaw() {
    return run(() -> io.setPivotVoltage(pivotTargetVoltage));
  }

  public Command goToPivotSetpoint() {
    return run(() -> io.goToPivotSetpoint());
  }

  public Command goToGrabberSetpoint() {
    return run(() -> io.goToGrabberSetpoint()).until(() -> io.grabberIsStable());
  }

  public Command stop() {
    return run(
        () -> {
          io.setGrabberVoltage(0);
          io.setPivotVoltage(0);
        });
  }

  public Command run() {
    return run(() -> io.goToGrabberSetpoint())
        .until(() -> io.grabberIsStable())
        .andThen(goToPivotSetpoint());
  }
}
