package org.curtinfrc.frc2025.subsystems.ejector;

import static org.curtinfrc.frc2025.subsystems.ejector.EjectorConstants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;

public class EjectorIONEO implements EjectorIO {
  private final SparkMax intakeNeo = new SparkMax(motorId, MotorType.kBrushless);
  private final DigitalInput sensor = new DigitalInput(sensorPort);

  public EjectorIONEO() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(0, currentLimit).idleMode(IdleMode.kCoast);
  }

  @Override
  public void updateInputs(EjectorIOInputs inputs) {
    inputs.appliedVolts = intakeNeo.getBusVoltage() * intakeNeo.getAppliedOutput();
    inputs.currentAmps = intakeNeo.getOutputCurrent();
    inputs.positionRotations = intakeNeo.getEncoder().getPosition();
    inputs.angularVelocityRotationsPerMinute = intakeNeo.getEncoder().getVelocity();
    inputs.sensor = sensor.get();
  }

  @Override
  public void setVoltage(double volts) {
    intakeNeo.setVoltage(volts);
  }
}
