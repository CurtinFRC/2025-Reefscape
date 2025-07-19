package org.curtinfrc.frc2025.subsystems.processor;

import static org.curtinfrc.frc2025.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import org.curtinfrc.frc2025.util.PhoenixUtil;

public class ProcessorIOComp implements ProcessorIO {
  private static final int ID = 20;
  private static final CurrentLimitsConfigs currentLimits =
      new CurrentLimitsConfigs().withSupplyCurrentLimit(20).withStatorCurrentLimit(40);

  private final TalonFX armMotor = new TalonFX(ID);
  private final TalonFX intakeMotor = new TalonFX(ID);

  private final DigitalInput processorSensor = new DigitalInput(999); // change later

  private final StatusSignal<Voltage> armVoltage = armMotor.getMotorVoltage();
  private final StatusSignal<Current> armCurrent = armMotor.getStatorCurrent();
  private final StatusSignal<Angle> armPosition = armMotor.getPosition();
  private final StatusSignal<AngularVelocity> armVelocity = armMotor.getVelocity();

  private final StatusSignal<Voltage> intakeVoltage = intakeMotor.getMotorVoltage();
  private final StatusSignal<Current> intakeCurrent = intakeMotor.getStatorCurrent();
  private final StatusSignal<Angle> intakePosition = intakeMotor.getPosition();
  private final StatusSignal<AngularVelocity> intakeVelocity = intakeMotor.getVelocity();

  private final VoltageOut armVoltageRequest = new VoltageOut(0).withEnableFOC(true);
  private final VoltageOut intakeVoltageRequest = new VoltageOut(0).withEnableFOC(true);

  public ProcessorIOComp() {
    tryUntilOk(
        5,
        () ->
            intakeMotor
                .getConfigurator()
                .apply(
                    new TalonFXConfiguration()
                        .withMotorOutput(
                            new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
                        .withCurrentLimits(currentLimits)));
    BaseStatusSignal.setUpdateFrequencyForAll(
        20.0, intakeVelocity, intakeVoltage, intakeCurrent, intakePosition);
    intakeMotor.optimizeBusUtilization();

    // Register signals to be updated
    PhoenixUtil.registerSignals(
        false, intakeVelocity, intakeVoltage, intakeCurrent, intakePosition);

    tryUntilOk(
        5,
        () ->
            armMotor
                .getConfigurator()
                .apply(
                    new TalonFXConfiguration()
                        .withMotorOutput(
                            new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
                        .withCurrentLimits(currentLimits)));
    BaseStatusSignal.setUpdateFrequencyForAll(
        20.0, armVelocity, armVoltage, armCurrent, armPosition);
    armMotor.optimizeBusUtilization();

    // Register signals to be updated
    PhoenixUtil.registerSignals(false, armVelocity, armVoltage, armCurrent, armPosition);
  }

  @Override
  public void updateInputs(ProcessorIOInputs inputs) {
    armMotor.setControl(armVoltageRequest);
    intakeMotor.setControl(intakeVoltageRequest);
    inputs.armAppliedVolts = armVoltage.getValueAsDouble();
    inputs.armCurrentAmps = armCurrent.getValueAsDouble();
    inputs.armPositionRotations = armPosition.getValueAsDouble();
    inputs.armAngularVelocityRotationsPerMinute = armVelocity.getValueAsDouble();
    inputs.intakeAppliedVolts = intakeVoltage.getValueAsDouble();
    inputs.intakeCurrentAmps = intakeCurrent.getValueAsDouble();
    inputs.intakePositionRotations = intakePosition.getValueAsDouble();
    inputs.intakeAngularVelocityRotationsPerMinute = intakeVelocity.getValueAsDouble();
    inputs.processorSensor = processorSensor.get();
  }

  @Override
  public void armSetVoltage(double volts) {
    armVoltageRequest.withOutput(volts);
  }

  @Override
  public void intakeSetVoltage(double volts) {
    intakeVoltageRequest.withOutput(volts);
  }
}
