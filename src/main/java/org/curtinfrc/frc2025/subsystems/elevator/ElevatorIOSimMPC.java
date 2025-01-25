package org.curtinfrc.frc2025.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.curtinfrc.frc2025.Constants.Setpoints;
import org.curtinfrc.frc2025.util.Util;

public class ElevatorIOSimMPC implements ElevatorIO {
  private final DCMotor motor = DCMotor.getNEO(ElevatorConstants.motorCount);
  private final DCMotorSim elevatorSim;
  private final double controlLoopPeriod = 0.02;

  private Setpoints set = Setpoints.COLLECT;
  private double computedVoltage = 0.0;
  private int predictionHorizon = 20;
  private double[] referenceTrajectory = new double[20];
  private double velocityError = 0;

  public ElevatorIOSimMPC() {
    elevatorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, 0.025, 10.7), motor);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    elevatorSim.update(controlLoopPeriod);

    inputs.distanceSensorReading = 0.0;
    inputs.encoderReading = elevatorSim.getAngularPositionRotations();
    inputs.point = set;
    inputs.pointRot = ElevatorIO.convertSetpoint(set.elevatorSetpoint());

    inputs.motorVoltage = elevatorSim.getInputVoltage();

    inputs.motorCurrent = elevatorSim.getCurrentDrawAmps();
    inputs.motorTemp = 25.0;
    inputs.motorVelocity = elevatorSim.getAngularVelocityRPM() / 60.0; // Convert RPM to RPS

    inputs.positionError =
        Math.abs(
            elevatorSim.getAngularPositionRotations()
                - ElevatorIO.convertSetpoint(set.elevatorSetpoint()));

    inputs.stable = isStable();

    inputs.predictionHorizon = predictionHorizon;

    inputs.predictedPosition = elevatorSim.getAngularPositionRotations();
    inputs.predictedVelocity = elevatorSim.getAngularVelocityRPM() / 60.0;

    inputs.velocityError = velocityError;
  }

  @Override
  public void goToSetpoint(Setpoints point) {
    set = point;

    double targetPosition = ElevatorIO.convertSetpoint(point.elevatorSetpoint());
    double currentPosition = elevatorSim.getAngularPositionRotations();
    double velocity = elevatorSim.getAngularVelocityRPM() / 60.0; // Convert RPM to RPS

    adjustPredictionHorizon(currentPosition, targetPosition);
    generateReferenceTrajectory(targetPosition);

    computedVoltage = computeMPCControl(currentPosition, velocity);

    computedVoltage =
        clampVoltageForDeadband(
            computedVoltage, Math.abs(targetPosition - currentPosition), velocity);

    // computedVoltage =
    // reduceVoltageNearSetpoint(computedVoltage, Math.abs(targetPosition - currentPosition));

    if (Math.abs(targetPosition - currentPosition) < ElevatorConstants.positionTolerance) {
      computedVoltage = 0;
    }

    elevatorSim.setInputVoltage(computedVoltage);
  }

  @Override
  public boolean isStable() {
    double positionError =
        Math.abs(
            elevatorSim.getAngularPositionRotations()
                - ElevatorIO.convertSetpoint(set.elevatorSetpoint()));
    double velocity = elevatorSim.getAngularVelocityRPM() / 60.0; // Convert RPM to RPS

    return positionError < ElevatorConstants.positionTolerance
        && Math.abs(velocity) < ElevatorConstants.stableVelocityThreshold;
  }

  private void generateReferenceTrajectory(double targetPosition) {
    double currentPosition = elevatorSim.getAngularPositionRotations();
    double distance = targetPosition - currentPosition;

    for (int i = 0; i < predictionHorizon; i++) {
      double factor = (double) i / (predictionHorizon - 1);
      referenceTrajectory[i] = currentPosition + distance * Math.pow(factor, 2);
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
          stepVoltage = 0;
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
        predictedVelocity[i] = 0;
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
      return 0.0;
    }
    return voltage;
  }

  private double reduceVoltageNearSetpoint(double voltage, double positionError) {
    if (positionError < ElevatorConstants.positionTolerance) {
      return 0.0;
    }
    double scalingFactor =
        Math.max(0.1, 1.0 - Math.abs(positionError) / (2 * ElevatorConstants.positionTolerance));
    return voltage * scalingFactor;
  }

  private double clampVoltage(double voltage) {
    return Math.max(ElevatorConstants.kMinOutput, Math.min(ElevatorConstants.kMaxOutput, voltage));
  }

  @Override
  public void stop() {
    elevatorSim.setInputVoltage(0);
  }
}
