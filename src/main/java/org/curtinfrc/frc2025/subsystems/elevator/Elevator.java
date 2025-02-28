package org.curtinfrc.frc2025.subsystems.elevator;

import static org.curtinfrc.frc2025.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.curtinfrc.frc2025.Constants.Setpoints;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final PIDController pid = new PIDController(kP, 0, kD);
  private final ElevatorFeedforward ff = new ElevatorFeedforward(kS, kG, kV, kA);
  private final TrapezoidProfile profile =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVelocity, maxAccel));
  private TrapezoidProfile.State previous_aim = new TrapezoidProfile.State(0, 0);
  private Setpoints setpoint = Setpoints.COLLECT;

  public final Trigger isNotAtCollect = new Trigger(() -> setpoint != Setpoints.COLLECT);
  public Trigger atSetpoint = new Trigger(pid::atSetpoint);

  public Elevator(ElevatorIO io) {
    this.io = io;
    pid.setTolerance(tolerance);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    Logger.recordOutput("Elevator/isNotAtCollect", isNotAtCollect.getAsBoolean());
    Logger.recordOutput("Elevator/setpoint", setpoint);
    Logger.recordOutput("Elevator/AtSetpoint", atSetpoint.getAsBoolean());
    Logger.recordOutput("Elevator/ActualError", pid.getError());
    Logger.recordOutput("Elevator/Position", positionRotationsToMetres(inputs.positionRotations));

    if (inputs.hominSensor) {
      io.zero();
    }
  }

  public Command goToSetpoint(Setpoints point) {
    return run(
        () -> {
          setpoint = point;
          var velocity = positionRotationsToMetres(inputs.angularVelocityRotationsPerMinute);
          Logger.recordOutput("Elevator/Velocity", velocity);
          Logger.recordOutput("Elevator/Setpoint", setpoint.elevatorSetpoint());
          var profileSetpoint =
              profile.calculate(
                  0.02, previous_aim, new TrapezoidProfile.State(setpoint.elevatorSetpoint(), 0));
          previous_aim = profileSetpoint;
          Logger.recordOutput("Elevator/ProfileSetpoint", profileSetpoint.position);
          var pid_out =
              pid.calculate(
                  positionRotationsToMetres(inputs.positionRotations), profileSetpoint.position);
          var ff_out = ff.calculate(profileSetpoint.velocity);
          Logger.recordOutput("Elevator/PIDOutput", pid_out);
          Logger.recordOutput("Elevator/FFOutput", ff_out);
          var out = pid_out + ff_out;
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
