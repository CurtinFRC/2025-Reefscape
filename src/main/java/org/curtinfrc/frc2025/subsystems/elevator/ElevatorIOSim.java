package org.curtinfrc.frc2025.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import org.curtinfrc.frc2025.util.FeedbackAnalysis;
import org.curtinfrc.frc2025.util.FeedbackAnalysis.FeedbackGains;

public class ElevatorIOSim implements ElevatorIO {
  private static final double KV = 3.35;
  private static final double KA = 0.02;
  private static final double KG = 0.12;

  private final LinearSystem<N2, N1, N2> elevatorSystem =
      LinearSystemId.identifyPositionSystem(KV, KA);
  private final ElevatorSim elevatorSim =
      new ElevatorSim(elevatorSystem, DCMotor.getKrakenX60Foc(2), 0, 0.62, true, 0);
  private final FeedbackGains gains =
      FeedbackAnalysis.getPositionGains(elevatorSystem, 12, 0.005, 0.1, 0.02, 0.02);
  private double volts = 0;

  private final TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(3.53, 48.74);
  private final TrapezoidProfile profile = new TrapezoidProfile(constraints);

  private final PIDController pid = new PIDController(gains.kP(), 0, gains.kD());
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
    var currentState =
        new TrapezoidProfile.State(
            elevatorSim.getPositionMeters(), elevatorSim.getVelocityMetersPerSecond());
    var setpoint =
        profile.calculate(0.02, currentState, new TrapezoidProfile.State(positionMetres, 0));
    setVoltage(
        pid.calculate(elevatorSim.getPositionMeters(), setpoint.position)
            + ff.calculateWithVelocities(
                elevatorSim.getVelocityMetersPerSecond(), setpoint.velocity));
  }
}
