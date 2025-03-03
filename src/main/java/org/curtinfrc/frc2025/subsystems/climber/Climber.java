package org.curtinfrc.frc2025.subsystems.climber;

import static org.curtinfrc.frc2025.subsystems.climber.ClimberConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private final PIDController pid = new PIDController(kP, kI, kD);

  public Climber(ClimberIO io) {
    this.io = io;
    pid.setTolerance(grabberTolerance);
    setDefaultCommand(run(() -> stop()));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    inputs.atSetpoint = pid.atSetpoint();
    Logger.processInputs("Climber", inputs);
  }

  public Command Raw() {
    return run(() -> io.setVoltage(4.0));
  }

  public Command goToSetpoint() {
    return run(() -> {
          var out = pid.calculate(inputs.positionRotations, targetPositionRotations);
          Logger.recordOutput("Climber/OutputVoltage", out);
          Logger.recordOutput("Climber/Error", pid.getError());
          io.setVoltage(out);
        })
        .until(pid::atSetpoint);
  }

  public Command stop() {
    return run(() -> io.setVoltage(0.0));
  }
}
