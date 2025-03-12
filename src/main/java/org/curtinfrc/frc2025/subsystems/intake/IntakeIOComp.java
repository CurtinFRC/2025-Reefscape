package org.curtinfrc.frc2025.subsystems.intake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import org.curtinfrc.frc2025.util.SparkUtil;

public class IntakeIOComp implements IntakeIO {
  private final SparkMax intakeNeo = new SparkMax(30, MotorType.kBrushless);
  private final DigitalInput frontSensor = new DigitalInput(3);
  private final DigitalInput backSensor = new DigitalInput(5);

  public IntakeIOComp() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(0, 35).idleMode(IdleMode.kCoast).inverted(true);
    SparkUtil.tryUntilOk(
        5,
        () ->
            intakeNeo.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
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
