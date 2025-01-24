package org.curtinfrc.frc2025.subsystems.intake;

import static org.curtinfrc.frc2025.subsystems.intake.IntakeConstants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIONEO implements IntakeIO {
  private final SparkMax intakeNeo = new SparkMax(intakeMotorId, MotorType.kBrushless);
  private final DigitalInput frontSensor = new DigitalInput(intakeFrontSensorPort);
  private final DigitalInput backSensor = new DigitalInput(intakeBackSensorPort);

  public IntakeIONEO() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(0, intakeCurrentLimit).idleMode(IdleMode.kCoast);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.appliedVolts = intakeNeo.getBusVoltage() * intakeNeo.getAppliedOutput();
    inputs.currentAmps = intakeNeo.getOutputCurrent();
    inputs.positionRotations = intakeNeo.getEncoder().getPosition();
    inputs.angularVelocityRotationsPerMinute = intakeNeo.getEncoder().getVelocity();
    inputs.frontSensor = frontSensor.get();
    inputs.backSensor = backSensor.get();
  }

  @Override
  public void setVoltage(double volts) {
    intakeNeo.setVoltage(volts);
  }
}
