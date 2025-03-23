package org.curtinfrc.frc2025.subsystems.intake;

import static org.curtinfrc.frc2025.subsystems.intake.IntakeConstants.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import org.curtinfrc.frc2025.util.SparkUtil;
import org.curtinfrc.frc2025.util.TestUtil;
import org.curtinfrc.frc2025.util.TestUtil.DigitalSensor;

public class IntakeIONEO implements IntakeIO {
  private final SparkMax intakeNeo = new SparkMax(intakeMotorId, MotorType.kBrushless);
  private final DigitalInput frontSensor = new DigitalInput(intakeFrontSensorPort);
  private final DigitalInput backSensor = new DigitalInput(intakeBackSensorPort);

  public IntakeIONEO(TestUtil tests) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(0, intakeCurrentLimit).idleMode(IdleMode.kCoast).inverted(true);
    SparkUtil.tryUntilOk(
        5,
        () ->
            intakeNeo.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    tests.addInput(tests.new DigitalSensor(frontSensor, "IntakeFront"));
    tests.addInput(tests.new DigitalSensor(backSensor, "IntakeBack"));
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
