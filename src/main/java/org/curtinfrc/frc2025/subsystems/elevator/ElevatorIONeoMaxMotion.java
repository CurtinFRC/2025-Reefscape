package org.curtinfrc.frc2025.subsystems.elevator;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;

import org.curtinfrc.frc2025.Constants.Setpoints;
import org.curtinfrc.frc2025.util.SparkUtil;

public class ElevatorIONeoMaxMotion extends ElevatorIONeo {
  private SparkClosedLoopController controller = elevatorMotor.getClosedLoopController();

  public static double convertSetpoint(double set /* in mm */) {
    return set / 21; /* mm to revolutions */
  }

  public ElevatorIONeoMaxMotion() {
    var config = new SparkMaxConfig();
    config.voltageCompensation(12.0).smartCurrentLimit(80);

    config
        .closedLoop
        .p(ElevatorConstants.kP)
        .i(ElevatorConstants.kI)
        .d(ElevatorConstants.kD)
        .outputRange(ElevatorConstants.kMinOutput, ElevatorConstants.kMaxOutput);

    config.closedLoop.velocityFF(1 / ElevatorConstants.kV);

    config
        .closedLoop
        .maxMotion
        .maxVelocity(ElevatorConstants.maxVel)
        .maxAcceleration(ElevatorConstants.maxAccel)
        .allowedClosedLoopError(ElevatorConstants.allowedErr);

    SparkUtil.tryUntilOk(
        elevatorMotor,
        5,
        () ->
            elevatorMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void goToSetpoint(Setpoints point) {
    setpoint = point;
    controller.setReference(convertSetpoint(point.elevatorSetpoint()), ControlType.kPosition);
  }

  @Override
  public boolean isStable() {
    double pos = elevatorEncoder.getPosition();
    return setpoint.elevatorSetpoint() - ElevatorConstants.tolerance < pos
        && pos < setpoint.elevatorSetpoint() + ElevatorConstants.tolerance;
  }
}
