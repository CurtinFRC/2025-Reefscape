package org.curtinfrc.frc2025.subsystems.elevator;

import static org.curtinfrc.frc2025.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
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

    // if (inputs.hominSensor) {
    //   io.zero();
    // }
  }

  public Command goToSetpoint(Supplier<ElevatorSetpoints> point, BooleanSupplier safe) {
    return Commands.either(
            run(
                () -> {
                  setpoint = point.get();
                  var out = pid.calculate(inputs.positionMetres, setpoint.setpoint);
                  Logger.recordOutput("Elevator/Output", out);
                  Logger.recordOutput("Elevator/Error", pid.getError());
                  Logger.recordOutput("Elevator/ClimberPID", false);
                  io.setVoltage(out);
                }),
            Commands.none(),
            safe)
        .repeatedly()
        .withName("GoToSetpoint");
  }

  public Command goToSetpoint(ElevatorSetpoints point, BooleanSupplier safe) {
    return Commands.either(
            run(
                () -> {
                  setpoint = point;
                  var out = pid.calculate(inputs.positionMetres, setpoint.setpoint);
                  Logger.recordOutput("Elevator/Output", out);
                  Logger.recordOutput("Elevator/Error", pid.getError());
                  Logger.recordOutput("Elevator/ClimberPID", false);
                  io.setVoltage(out);
                }),
            Commands.none(),
            safe)
        .repeatedly()
        .withName("GoToSetpoint");
  }

  public Command goToClimberSetpoint(ElevatorSetpoints point, BooleanSupplier safe) {
    return Commands.either(
        run(
            () -> {
              setpoint = point;
              var out = climbPID.calculate(inputs.positionMetres, setpoint.setpoint);
              io.setVoltage(MathUtil.clamp(out, -4, 4));
            }),
        Commands.none(),
        safe);
  }

  public Command zero() {
    return runOnce(() -> io.zero());
  }

  public Command stop() {
    return runOnce(() -> io.setVoltage(0));
  }

  @AutoLogOutput(key = "Elevator/Height")
  public Pose3d getHeight() {
    return new Pose3d(0, 0, inputs.positionMetres, new Rotation3d(Math.PI / 2, 0, Math.PI / 2));
  }
}
