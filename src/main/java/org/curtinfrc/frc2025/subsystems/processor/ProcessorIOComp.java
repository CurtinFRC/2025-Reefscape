package org.curtinfrc.frc2025.subsystems.processor;

import static org.curtinfrc.frc2025.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import org.curtinfrc.frc2025.util.PhoenixUtil;

public class ProcessorIOComp implements ProcessorIO {
  private static final int armMotorID = 1;
  private static final int armEncoderID = 0;
  private static final int intakeMotorID = 29;
  private static final int intakeEncoderID = 0;

  private static final CurrentLimitsConfigs currentLimits =
      new CurrentLimitsConfigs().withSupplyCurrentLimit(20).withStatorCurrentLimit(50);

  private final TalonFX armMotor = new TalonFX(armMotorID);
  private final CANcoder armEncoder = new CANcoder(armEncoderID);
  private final TalonFX intakeMotor = new TalonFX(intakeMotorID);
  private final CANcoder intakeEncoder = new CANcoder(intakeEncoderID);
  private final DigitalInput processorSensor = new DigitalInput(6);

  // private final StatusSignal<Angle> armAbsolutePosition =
  // armAbsoluteEncoder.getAbsolutePosition();
  private final StatusSignal<Voltage> armVoltage = armMotor.getMotorVoltage();
  private final StatusSignal<Current> armCurrent = armMotor.getStatorCurrent();
  private final StatusSignal<Angle> armPosition = armMotor.getPosition();
  private final StatusSignal<Angle> armAbsolutePosition = armEncoder.getAbsolutePosition();
  private final StatusSignal<AngularVelocity> armVelocity = armMotor.getVelocity();

  private final StatusSignal<Voltage> intakeVoltage = intakeMotor.getMotorVoltage();
  private final StatusSignal<Current> intakeCurrent = intakeMotor.getStatorCurrent();
  private final StatusSignal<Angle> intakePosition = intakeMotor.getPosition();
  private final StatusSignal<Angle> intakeAbsolutePosition = intakeEncoder.getAbsolutePosition();
  private final StatusSignal<AngularVelocity> intakeVelocity = intakeMotor.getVelocity();

  private final VoltageOut armVoltageRequest = new VoltageOut(0).withEnableFOC(true);
  private final VoltageOut intakeVoltageRequest = new VoltageOut(0).withEnableFOC(true);

  public ProcessorIOComp() {
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
        20.0, armVelocity, armVoltage, armCurrent, armPosition, armAbsolutePosition);
    armMotor.optimizeBusUtilization();

    // Register signals to be updated
    PhoenixUtil.registerSignals(
        false, armVelocity, armVoltage, armCurrent, armPosition, armAbsolutePosition);

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
        20.0, intakeVelocity, intakeVoltage, intakeCurrent, intakePosition, intakeAbsolutePosition);
    intakeMotor.optimizeBusUtilization();

    intakeEncoder
        .getConfigurator()
        .apply(
            new MagnetSensorConfigs()
                .withMagnetOffset(0.487)
                .withSensorDirection(SensorDirectionValue.Clockwise_Positive));

    // Register signals to be updated
    PhoenixUtil.registerSignals(
        false,
        intakeVelocity,
        intakeVoltage,
        intakeCurrent,
        intakePosition,
        intakeAbsolutePosition);
  }

  @Override
  public void updateInputs(ProcessorIOInputs inputs) {
    armMotor.setControl(armVoltageRequest);
    intakeMotor.setControl(intakeVoltageRequest);
    inputs.armAppliedVolts = armVoltage.getValueAsDouble();
    inputs.armCurrentAmps = armCurrent.getValueAsDouble();
    inputs.armPositionRotations = armPosition.getValueAsDouble();
    inputs.armAbsolutePosition = armAbsolutePosition.getValueAsDouble();
    inputs.armAngularVelocityRotationsPerMinute = armVelocity.getValueAsDouble();
    inputs.intakeAppliedVolts = intakeVoltage.getValueAsDouble();
    inputs.intakeCurrentAmps = intakeCurrent.getValueAsDouble();
    inputs.intakePositionRotations = intakePosition.getValueAsDouble();
    inputs.intakeAngularVelocityRotationsPerMinute = intakeAbsolutePosition.getValueAsDouble();
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
