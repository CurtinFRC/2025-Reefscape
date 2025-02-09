package org.curtinfrc.frc2025.subsystems.ejector;

import static org.curtinfrc.frc2025.subsystems.ejector.EjectorConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

public class Ejector extends SubsystemBase {
  private final EjectorIO io;
  private final EjectorIOInputsAutoLogged inputs = new EjectorIOInputsAutoLogged();
  private final PIDController pid = new PIDController(kP, 0, kD);
  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kS, kV, kA);

  public Ejector(EjectorIO io) {
    this.io = io;
  }

  public final Trigger sensor = new Trigger(() -> inputs.sensor);
  public final Trigger atSetpoint = new Trigger(pid::atSetpoint);

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Ejector", inputs);
  }

  public Command stop() {
    return runOnce(() -> io.setVoltage(0));
  }

  public Command eject(double rpm) {
    return run(
        () -> {
          Logger.recordOutput("Ejector/VelocitySetpoint", rpm);
          var pid_out = pid.calculate(inputs.angularVelocityRotationsPerMinute, rpm);
          Logger.recordOutput("Ejector/VelocityErrror", pid.getError());
          var ff_out = ff.calculate(rpm);
          io.setVoltage(pid_out + ff_out);
        });
  }
}
