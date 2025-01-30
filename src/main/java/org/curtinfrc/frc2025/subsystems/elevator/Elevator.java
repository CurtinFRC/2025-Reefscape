package org.curtinfrc.frc2025.subsystems.elevator;

import static org.curtinfrc.frc2025.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.curtinfrc.frc2025.Constants.Setpoints;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final PIDController pid = new PIDController(kP, 0, kD);
  private Setpoints setpoint = Setpoints.COLLECT;

  public final Trigger isNotAtCollect = new Trigger(() -> setpoint != Setpoints.COLLECT);

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    Logger.recordOutput("Elevator/setpoint", setpoint);
    Logger.recordOutput("Elevator/isNotAtCollect", isNotAtCollect);
  }

  public Trigger atSetpoint = new Trigger(pid::atSetpoint);

  public Command goToSetpoint(Setpoints point) {
    setpoint = point;
    return run(
        () -> {
          var out = pid.calculate(inputs.positionRotations, point.elevatorSetpoint());
          Logger.recordOutput("Elevator/Output", out);
          Logger.recordOutput("Elevator/Error", pid.getError());
          io.setVoltage(out);
        });
  }

  public Command stop() {
    return runOnce(() -> io.setVoltage(0));
  }
}
