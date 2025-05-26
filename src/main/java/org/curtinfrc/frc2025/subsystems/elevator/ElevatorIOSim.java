package org.curtinfrc.frc2025.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
  private static final double KV = 3.35;
  private static final double KA = 0.02;
  private static final double KG = 0.12;

  private final ElevatorSim elevatorSim =
      new ElevatorSim(KV, KA, DCMotor.getKrakenX60Foc(2), 0, 0.62, true, 0);
  private double volts = 0;

  private final TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(3.53, 48.74);
  private final TrapezoidProfile profile = new TrapezoidProfile(constraints);

  private final PIDController pid = new PIDController(0, 0, 0);
  private final ElevatorFeedforward ff = new ElevatorFeedforward(0, KG, KV, KA);

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    elevatorSim.update(0.02);
    inputs.currentAmps = elevatorSim.getCurrentDrawAmps();
    inputs.appliedVolts = volts;
    inputs.positionMetres = elevatorSim.getPositionMeters();
    inputs.velocityMetresPerSecond = elevatorSim.getVelocityMetersPerSecond();
  }

  @Override
  public void setVoltage(double volts) {
    this.volts = volts;
    elevatorSim.setInputVoltage(volts);
  }

  @Override
  public void setPosition(double positionMetres) {
    var setpoint =
        profile.calculate(
            0.02,
            new TrapezoidProfile.State(
                elevatorSim.getPositionMeters(), elevatorSim.getVelocityMetersPerSecond()),
            new TrapezoidProfile.State(positionMetres, 0));
    setVoltage(
        pid.calculate(elevatorSim.getPositionMeters(), setpoint.position)
            + ff.calculateWithVelocities(
                elevatorSim.getVelocityMetersPerSecond(), setpoint.velocity));
  }
}
