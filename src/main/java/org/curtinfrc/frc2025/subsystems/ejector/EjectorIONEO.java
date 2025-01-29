package org.curtinfrc.frc2025.subsystems.ejector;

import static org.curtinfrc.frc2025.subsystems.ejector.EjectorConstants.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.curtinfrc.frc2025.util.SparkUtil;

public class EjectorIONEO implements EjectorIO {
  private final SparkMax neo = new SparkMax(motorId, MotorType.kBrushless);
  // private final DigitalInput sensor = new DigitalInput(sensorPort);

  public EjectorIONEO() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(0, currentLimit).idleMode(IdleMode.kCoast);

    SparkUtil.tryUntilOk(
        5,
        () ->
            neo.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(EjectorIOInputs inputs) {
    inputs.appliedVolts = neo.getBusVoltage() * neo.getAppliedOutput();
    inputs.currentAmps = neo.getOutputCurrent();
    inputs.positionRotations = neo.getEncoder().getPosition();
    inputs.angularVelocityRotationsPerMinute = neo.getEncoder().getVelocity();
    // inputs.sensor = sensor.get();
  }

  @Override
  public void setVoltage(double volts) {
    neo.setVoltage(volts);
  }
}
