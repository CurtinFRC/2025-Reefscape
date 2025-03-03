package org.curtinfrc.frc2025.subsystems.elevator;

import static org.curtinfrc.frc2025.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.curtinfrc.frc2025.subsystems.elevator.ElevatorConstants.ElevatorSetpoints;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final PIDController pid = new PIDController(kP, 0, kD);
  private final PIDController climbPID = new PIDController(climbkP, climbkI, climbkD);
  private ElevatorSetpoints setpoint = ElevatorSetpoints.BASE;

  public final Trigger isNotAtCollect = new Trigger(() -> setpoint != ElevatorSetpoints.BASE);
  public final Trigger toZero = new Trigger(() -> inputs.hominSensor);
  public final Trigger atSetpoint = new Trigger(pid::atSetpoint);
  public final Trigger atClimbSetpoint = new Trigger(climbPID::atSetpoint);
  public final Trigger algaePop =
      new Trigger(
          () ->
              setpoint == ElevatorSetpoints.AlgaePopHigh
                  || setpoint == ElevatorSetpoints.AlgaePopLow);

  public Elevator(ElevatorIO io) {
    this.io = io;
    pid.setTolerance(tolerance);
    climbPID.setTolerance(tolerance);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    Logger.recordOutput("Elevator/isNotAtCollect", isNotAtCollect.getAsBoolean());
    Logger.recordOutput("Elevator/setpoint", ElevatorSetpoints.struct, setpoint);
    Logger.recordOutput("Elevator/AtSetpoint", atSetpoint.getAsBoolean());
    Logger.recordOutput("Elevator/ActualError", pid.getError());

    if (inputs.hominSensor) {
      io.zero();
    }
  }

  public Command goToSetpoint(ElevatorSetpoints point) {
    return run(
        () -> {
          setpoint = point;
          var out =
              pid.calculate(positionRotationsToMetres(inputs.positionRotations), setpoint.setpoint);
          Logger.recordOutput("Elevator/Output", out);
          Logger.recordOutput("Elevator/Error", pid.getError());
          Logger.recordOutput("Elevator/ClimberPID", false);
          io.setVoltage(out);
        });
  }

  public Command goToClimberSetpoint(ElevatorSetpoints point) {
    return run(
        () -> {
          setpoint = point;
          var out =
              climbPID.calculate(
                  positionRotationsToMetres(inputs.positionRotations), setpoint.setpoint);
          Logger.recordOutput("Elevator/ClimberOutput", out);
          Logger.recordOutput("Elevator/ClimberError", pid.getError());
          Logger.recordOutput("Elevator/ClimberPID", true);
          io.setVoltage(MathUtil.clamp(out, -3, 3));
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
