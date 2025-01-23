package org.curtinfrc.frc2025.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import org.curtinfrc.frc2025.Constants.Setpoints;
import org.curtinfrc.frc2025.util.Util;

/** Simulated Elevator implementation using Model Predictive Control (MPC). */
public class ElevatorIONeoMPC extends ElevatorIONeo {
  private final double controlLoopPeriod = 0.02; // 50 Hz simulation step

  private Setpoints set = Setpoints.COLLECT;
  private double computedVoltage = 0.0;
  private int predictionHorizon = 20;
  private double[] referenceTrajectory = new double[20];
  private double velocityError = 0;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // Simulate sensor readings and MPC state
    inputs.distanceSensorReading = 0.0; // Distance sensor is not used in this simulation
    inputs.encoderReading = elevatorEncoder.getPosition(); // Simulated encoder reading
    inputs.point = set; // Current setpoint
    inputs.pointRot =
        ElevatorIONeoMaxMotion.convertSetpoint(set.elevatorSetpoint()); // Setpoint in rotations

    // Voltage applied to the motor (computed by the MPC)
    inputs.motorVoltage = computedVoltage;

    // Simulated motor properties
    inputs.motorCurrent = elevatorMotor.getOutputCurrent(); // Simulated motor current
    inputs.motorTemp = 25.0; // Assume constant motor temperature for the simulation
    inputs.motorVelocity = elevatorEncoder.getVelocity() / 60.0; // Convert RPM to RPS

    // Position error: difference between current position and target
    inputs.positionError =
        Math.abs(
            elevatorEncoder.getPosition()
                - ElevatorIONeoMaxMotion.convertSetpoint(set.elevatorSetpoint()));

    // Stability check based on position and velocity thresholds
    inputs.stable = isStable();

    // Prediction horizon used by the MPC
    inputs.predictionHorizon = predictionHorizon;

    // Fallback PID status (disabled in this simulation)
    inputs.useFallbackPID = false;

    // Simulated predicted position and velocity
    inputs.predictedPosition =
        elevatorEncoder.getPosition(); // Predicted position from simulation
    inputs.predictedVelocity =
        elevatorEncoder.getVelocity() / 60.0; // Predicted velocity in RPS

    // Velocity error (difference between desired and actual velocity)
    inputs.velocityError = velocityError;
  }

  @Override
  public void goToSetpoint(Setpoints point) {
    set = point;

    double targetPosition = ElevatorIONeoMaxMotion.convertSetpoint(point.elevatorSetpoint());
    double currentPosition = elevatorEncoder.getPosition();
    double velocity = elevatorEncoder.getVelocity() / 60.0; // Convert RPM to RPS

    adjustPredictionHorizon(currentPosition, targetPosition);
    generateReferenceTrajectory(targetPosition);

    computedVoltage = computeMPCControl(currentPosition, velocity);

    // Apply clamping and deadband logic
    computedVoltage =
        clampVoltageForDeadband(
            computedVoltage, Math.abs(targetPosition - currentPosition), velocity);

    // computedVoltage =
    // reduceVoltageNearSetpoint(computedVoltage, Math.abs(targetPosition - currentPosition));

    if (Math.abs(targetPosition - currentPosition) < ElevatorConstants.positionTolerance) {
      computedVoltage = 0; // Force motor to stop
    }

    elevatorMotor.setVoltage(computedVoltage);
  }

  @Override
  public boolean isStable() {
    double positionError =
        Math.abs(
            elevatorEncoder.getPosition()
                - ElevatorIONeoMaxMotion.convertSetpoint(set.elevatorSetpoint()));
    double velocity = elevatorEncoder.getVelocity() / 60.0; // Convert RPM to RPS

    return positionError < ElevatorConstants.positionTolerance
        && Math.abs(velocity) < ElevatorConstants.stableVelocityThreshold;
  }

  private void generateReferenceTrajectory(double targetPosition) {
    double currentPosition = elevatorEncoder.getPosition();
    double distance = targetPosition - currentPosition;

    for (int i = 0; i < predictionHorizon; i++) {
      double factor = (double) i / (predictionHorizon - 1);
      referenceTrajectory[i] =
          currentPosition + distance * Math.pow(factor, 2); // Quadratic slowdown
    }
  }

  private double computeMPCControl(double currentPosition, double velocity) {
    double kP = ElevatorConstants.kP;
    double kD = ElevatorConstants.kD;
    double kFF = 1 / ElevatorConstants.kV;

    double[] predictedPosition = new double[predictionHorizon];
    double[] predictedVelocity = new double[predictionHorizon];

    predictedPosition[0] = currentPosition;
    predictedVelocity[0] = velocity;

    double totalVoltage = 0.0;

    for (int i = 1; i < predictionHorizon; i++) {
      double positionError = referenceTrajectory[i] - predictedPosition[i - 1];
      velocityError =
          (referenceTrajectory[i] - referenceTrajectory[i - 1]) / controlLoopPeriod
              - predictedVelocity[i - 1];
      velocityError =
          Util.clamp(-ElevatorConstants.maxVel, ElevatorConstants.maxVel, velocityError);

      double stepVoltage = kP * positionError + kD * velocityError;

      if (Math.abs(positionError) < ElevatorConstants.positionTolerance) {
        stepVoltage *= Math.abs(positionError) / ElevatorConstants.positionTolerance;
        if (Math.abs(stepVoltage) < 0.1) {
          stepVoltage = 0; // Stop applying voltage when near the setpoint
        }
      }

      predictedVelocity[i] =
          predictedVelocity[i - 1]
              + ((stepVoltage - ElevatorConstants.friction * predictedVelocity[i - 1])
                      / ElevatorConstants.kV)
                  * ElevatorConstants.kA
                  * controlLoopPeriod;

      predictedPosition[i] =
          predictedPosition[i - 1] + predictedVelocity[i - 1] * controlLoopPeriod;

      if (Math.abs(predictedPosition[i] - referenceTrajectory[i])
          < ElevatorConstants.positionTolerance) {
        predictedVelocity[i] = 0; // Stop when the position is close to the target
      }

      totalVoltage = stepVoltage;
      totalVoltage +=
          kFF * velocity
              + ElevatorConstants.kA
                  * (predictedVelocity[i] - predictedVelocity[i - 1])
                  / controlLoopPeriod;
    }

    totalVoltage += kFF * velocity;
    return clampVoltage(totalVoltage);
  }

  private void adjustPredictionHorizon(double currentPosition, double targetPosition) {
    double distanceToTarget = Math.abs(targetPosition - currentPosition);
    predictionHorizon = (distanceToTarget < 0.1) ? 5 : (distanceToTarget < 0.5) ? 10 : 20;
    referenceTrajectory = new double[predictionHorizon];
  }

  private boolean isWithinDeadband(double positionError, double velocity) {
    return Math.abs(positionError) < (0.5 * ElevatorConstants.positionTolerance)
        && Math.abs(velocity) < (0.5 * ElevatorConstants.stableVelocityThreshold);
  }

  private double clampVoltageForDeadband(double voltage, double positionError, double velocity) {
    if (isWithinDeadband(positionError, velocity)) {
      return 0.0; // Stop applying voltage when within deadband
    }
    return voltage;
  }

  private double reduceVoltageNearSetpoint(double voltage, double positionError) {
    if (positionError < ElevatorConstants.positionTolerance) {
      return 0.0; // Stop voltage entirely near the setpoint
    }
    double scalingFactor =
        Math.max(0.1, 1.0 - Math.abs(positionError) / (2 * ElevatorConstants.positionTolerance));
    return voltage * scalingFactor; // Scale voltage more aggressively near setpoint
  }

  private double clampVoltage(double voltage) {
    return Math.max(ElevatorConstants.kMinOutput, Math.min(ElevatorConstants.kMaxOutput, voltage));
  }
}
