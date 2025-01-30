package org.curtinfrc.frc2025.subsystems.elevator;

import static org.curtinfrc.frc2025.subsystems.ejector.EjectorConstants.currentLimit;
import static org.curtinfrc.frc2025.subsystems.elevator.ElevatorConstants.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.curtinfrc.frc2025.util.SparkUtil;

public class ElevatorIONEO implements ElevatorIO {
  private final SparkMax neo = new SparkMax(motorPort, MotorType.kBrushless);
  // private final DigitalInput sensor = new DigitalInput(sensorPort);

  public ElevatorIONEO() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(0, currentLimit).idleMode(IdleMode.kCoast).inverted(true);

    SparkUtil.tryUntilOk(
        5,
        () ->
            neo.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.appliedVolts = neo.getBusVoltage() * neo.getAppliedOutput();
    inputs.currentAmps = neo.getOutputCurrent();
    inputs.positionRotations = neo.getEncoder().getPosition();
    inputs.angularVelocityRotationsPerMinute = neo.getEncoder().getVelocity();
  }

  @Override
  public void setVoltage(double volts) {
    neo.setVoltage(volts);
  }
}
