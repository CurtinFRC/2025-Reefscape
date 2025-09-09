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
  private final PIDController climbPID = new PIDController(45, 0, 0);
  private ElevatorSetpoints setpoint = ElevatorSetpoints.BASE;

  public final Trigger isNotAtCollect = new Trigger(() -> setpoint != ElevatorSetpoints.BASE);

  public final Trigger isNotAtCollectOrL1 =
      new Trigger(() -> setpoint != ElevatorSetpoints.BASE && setpoint != ElevatorSetpoints.L1);

  public final Trigger toZero = new Trigger(() -> inputs.hominSensor);
  public final Trigger atSetpoint;

  @AutoLogOutput(key = "Climber/ElevatorAtSetpoint")
  public final Trigger atClimbSetpoint = new Trigger(climbPID::atSetpoint);

  public final Trigger algaePop =
      new Trigger(
          () ->
              setpoint == ElevatorSetpoints.AlgaePopHigh
                  || setpoint == ElevatorSetpoints.AlgaePopLow);

  public Elevator(ElevatorIO io) {
    this.io = io;
    climbPID.setTolerance(0.01);

    atSetpoint =
        new Trigger(
            () -> {
              var referencePos = io.metresToRotations(setpoint.setpoint);
              var posTolerance = io.metresToRotations(0.005);
              var currentPos = inputs.positionRotations;

              var velTolerance = io.metresToRotations(0.3);
              var currentVel = inputs.angularVelocityRotationsPerSecond;

              return Math.abs(referencePos - currentPos) < posTolerance
                  && Math.abs(0 - currentVel) < velTolerance;
            });
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    Logger.recordOutput("Elevator/isNotAtCollect", isNotAtCollect.getAsBoolean());
    Logger.recordOutput("Elevator/setpoint", ElevatorSetpoints.struct, setpoint);
    Logger.recordOutput("Elevator/AtSetpoint", atSetpoint.getAsBoolean());

    // if (inputs.hominSensor) {
    //   io.zero();
    // }
  }

  public Command goToSetpoint(Supplier<ElevatorSetpoints> point, BooleanSupplier safe) {
    return Commands.either(
            run(
                () -> {
                  setpoint = point.get();
                  io.setPosition(setpoint.setpoint);
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
                  io.setPosition(setpoint.setpoint);
                }),
            Commands.none(),
            safe)
        .repeatedly()
        .withName("GoToSetpoint " + point.toString());
  }

  public Command goToClimberSetpoint(ElevatorSetpoints point, BooleanSupplier safe) {
    return Commands.either(
        run(
            () -> {
              setpoint = point;
              var out =
                  climbPID.calculate(
                      io.rotationsToMetres(inputs.positionRotations), setpoint.setpoint);
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
    return new Pose3d(
        0,
        0,
        io.rotationsToMetres(inputs.positionRotations),
        new Rotation3d(Math.PI / 2, 0, Math.PI / 2));
  }
}
