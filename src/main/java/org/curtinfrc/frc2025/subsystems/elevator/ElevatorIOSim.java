package org.curtinfrc.frc2025.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim extends ElevatorIOComp {
  private final ElevatorSim elevatorSim =
      new ElevatorSim(5.29, 0.01, DCMotor.getKrakenX60Foc(2), 0, 0.7, true, 0, 0.0005, 0.005);

  private double lastSimTime;
  private final Notifier notifier;

  public ElevatorIOSim() {
    super();
    lastSimTime = RobotController.getFPGATime();

    notifier =
        new Notifier(
            () -> {
              final double currentTime = RobotController.getFPGATime();
              double deltaTime = currentTime - lastSimTime;
              lastSimTime = currentTime;

              updateSimState(deltaTime);
            });
    notifier.startPeriodic(0.001);
  }

  private void updateSimState(double deltaTime) {
    var simState = motor.getSimState();
    elevatorSim.setInput(simState.getMotorVoltage());
    elevatorSim.update(0.02);
    var rotorPosition = metresToRotations(elevatorSim.getPositionMeters());
    simState.setRawRotorPosition(rotorPosition);
    var rotorVel = metresToRotations(elevatorSim.getVelocityMetersPerSecond());
    simState.setRotorVelocity(rotorVel);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    super.updateInputs(inputs);
    final double currentTime = RobotController.getFPGATime();
    double deltaTime = currentTime - lastSimTime;
    lastSimTime = currentTime;
    updateSimState(deltaTime);
  }
}
