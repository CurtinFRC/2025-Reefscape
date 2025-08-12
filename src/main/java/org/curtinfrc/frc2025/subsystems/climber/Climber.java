package org.curtinfrc.frc2025.subsystems.climber;

import static org.curtinfrc.frc2025.subsystems.climber.ClimberConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private final PIDController pid = new PIDController(kP.get(), kI.get(), kD.get());
  public boolean climberDeployed = false;

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

    if (kP.hasChanged(kP.hashCode())) {
      pid.setP(kP.get());
    }
    if (kI.hasChanged(kI.hashCode())) {
      pid.setI(kI.get());
    }
    if (kD.hasChanged(kD.hashCode())) {
      pid.setD(kD.get());
    }
  }

  public Command Raw() {
    return run(() -> io.setVoltage(4.0));
  }

  public Command goToSetpoint(double targetPositionRotations) {
    return run(() -> {
          var out = pid.calculate(inputs.positionRotations, targetPositionRotations);
          Logger.recordOutput("Climber/OutputVoltage", out);
          Logger.recordOutput("Climber/Error", pid.getError());
          io.setVoltage(out);
          Logger.recordOutput("Climber/TargetPosition", targetPositionRotations);
          if (Math.abs(inputs.positionRotations - ClimberConstants.targetPositionRotationsIn)
              < ClimberConstants.deployTolerance) {
            climberDeployed = true;
          }
          Logger.recordOutput("Climber/climberDeployed", climberDeployed);
        })
        .until(pid::atSetpoint);
  }

  public Command stop() {
    return runOnce(() -> io.setVoltage(0.0));
  }

  public Command engage() {
    return runOnce(() -> io.engageRatchet());
  }

  public Command disengage() {
    return runOnce(() -> io.disableRatchet());
  }

  @AutoLogOutput(key = "Climber/stalled")
  public final Trigger stalled =
      new Trigger(
              () ->
                  inputs.currentAmps > stallingCurrent
                      && inputs.angularVelocityRotationsPerMinute < stallingRPM)
          .debounce(0.1);
}
