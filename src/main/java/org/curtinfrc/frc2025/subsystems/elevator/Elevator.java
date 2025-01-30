package org.curtinfrc.frc2025.subsystems.elevator;

import static org.curtinfrc.frc2025.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.curtinfrc.frc2025.Constants;
import org.curtinfrc.frc2025.Constants.Setpoints;
import org.curtinfrc.frc2025.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final PIDController pid = new PIDController(0, 0, 0);

  @AutoLogOutput(key = "Elevator/Setpoint")
  private Setpoints setpoint = Setpoints.COLLECT;

  // Tunable Numbers
  LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP");
  LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD");

  @AutoLogOutput(key = "Elevator/IsNotAtCollect")
  public final Trigger isNotAtCollect = new Trigger(() -> setpoint != Setpoints.COLLECT);

  public final Trigger toZero = new Trigger(() -> inputs.touchSensor);

  @AutoLogOutput(key = "Elevator/AtSetpoint")
  public Trigger atSetpoint = new Trigger(pid::atSetpoint);

  public Elevator(ElevatorIO io) {
    this.io = io;
    pid.setTolerance(tolerance);
    switch (Constants.robotType) {
      case COMPBOT:
        kP.initDefault(16);
        kD.initDefault(0);
        pid.setPID(kP.get(), 0, kD.get());
        break;
      case DEVBOT:
        kP.initDefault(16);
        kD.initDefault(0);
        pid.setPID(kP.get(), 0, kD.get());
        break;
      case SIMBOT:
        kP.initDefault(16);
        kD.initDefault(0);
        pid.setPID(kP.get(), 0, kD.get());
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    Logger.recordOutput("Elevator/isNotAtCollect", isNotAtCollect.getAsBoolean());
    Logger.recordOutput("Elevator/setpoint", setpoint);
    Logger.recordOutput("Elevator/AtSetpoint", atSetpoint.getAsBoolean());
    Logger.recordOutput("Elevator/ActualError", pid.getError());

    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      pid.setPID(kP.get(), 0, kD.get());
    }
  }

  public Command goToSetpoint(Setpoints point) {
    return run(
        () -> {
          setpoint = point;
          var out =
              pid.calculate(
                  positionRotationsToMetres(inputs.positionRotations), setpoint.elevatorSetpoint());
          Logger.recordOutput("Elevator/Output", out);
          Logger.recordOutput("Elevator/Error", pid.getError());
          io.setVoltage(out);
        });
  }

  public Command zero() {
    return runOnce(() -> io.zero());
  }

  public Command stop() {
    return runOnce(() -> io.setVoltage(0));
  }

  @AutoLogOutput(key = "Elevator/Height")
  public Pose3d getHeight() {
    return new Pose3d(
        0,
        0,
        positionRotationsToMetres(inputs.positionRotations),
        new Rotation3d(Math.PI / 2, 0, Math.PI / 2));
  }
}
