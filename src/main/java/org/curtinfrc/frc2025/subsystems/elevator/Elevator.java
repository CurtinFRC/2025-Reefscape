package org.curtinfrc.frc2025.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private static final double TOLERANCE = 0.005;
  public static double CLIMB_KP = 45;
  public static double CLIMB_KD = 0;

  public static enum ElevatorSetpoints implements StructSerializable {
    L1(0),
    L2(0.221),
    AlgaePopLow(0),
    L3(0.611),
    AlgaePopHigh(0.38),
    BASE(0.01),
    climbPrep(0.3),
    climbAttempt(0.5),
    climbed(0.01);

    public final double setpointMetres;

    ElevatorSetpoints(double setpointMetres) {
      this.setpointMetres = setpointMetres;
    }

    public static ElevatorSetpoints getPopPoint(ElevatorSetpoints point) {
      switch (point) {
        case L2:
          return AlgaePopLow;
        case L3:
          return AlgaePopHigh;
        default:
          return BASE;
      }
    }

    public static final Struct<ElevatorSetpoints> struct =
        StructGenerator.genEnum(ElevatorSetpoints.class);
  }

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final PIDController climbPID = new PIDController(CLIMB_KP, 0, CLIMB_KD);
  private ElevatorSetpoints setpoint = ElevatorSetpoints.BASE;

  public final Trigger isNotAtCollect = new Trigger(() -> setpoint != ElevatorSetpoints.BASE);
  public final Trigger atSetpoint;
  public final Trigger atClimbSetpoint = new Trigger(climbPID::atSetpoint);

  public Elevator(ElevatorIO io) {
    this.io = io;
    climbPID.setTolerance(TOLERANCE);

    atSetpoint =
        new Trigger(
                () -> {
                  var error = Math.abs(inputs.positionMetres - setpoint.setpointMetres);
                  var velocity = Math.abs(inputs.velocityMetresPerSecond);
                  return error < TOLERANCE && velocity < 0.1;
                })
            .debounce(0.01);
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

  private void goToTarget(double targetMetres) {
    io.setPosition(targetMetres);
  }

  public Command goToSetpoint(Supplier<ElevatorSetpoints> point, BooleanSupplier safe) {
    return run(() -> {
          setpoint = point.get();
          if (safe.getAsBoolean()) {
            goToTarget(point.get().setpointMetres);
          }
        })
        .withName("GoToDynamicSetpoint");
  }

  public Command goToSetpoint(ElevatorSetpoints point, BooleanSupplier safe) {
    return run(() -> {
          setpoint = point;
          if (safe.getAsBoolean()) {
            goToTarget(point.setpointMetres);
          }
        })
        .withName("GoToStaticSetpoint");
  }

  public Command goToClimberSetpoint(ElevatorSetpoints point, BooleanSupplier safe) {
    return run(
        () -> {
          if (safe.getAsBoolean()) {
            var out = climbPID.calculate(inputs.positionMetres, point.setpointMetres);
            io.setVoltage(MathUtil.clamp(out, -4, 4));
          }
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
    return new Pose3d(0, 0, inputs.positionMetres, new Rotation3d(Math.PI / 2, 0, Math.PI / 2));
  }
}
