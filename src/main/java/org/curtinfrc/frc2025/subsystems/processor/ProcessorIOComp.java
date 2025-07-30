package org.curtinfrc.frc2025.subsystems.processor;

import static org.curtinfrc.frc2025.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;

import org.curtinfrc.frc2025.util.PhoenixUtil;

public class ProcessorIOComp implements ProcessorIO {
  private static final int ID = 0;
  private static final CurrentLimitsConfigs currentLimits =
      new CurrentLimitsConfigs().withSupplyCurrentLimit(20).withStatorCurrentLimit(40);
    
  private final TalonFX armMotor = new TalonFX(ID); // change later
  private final SparkMax intakeMotor = new SparkMax(0, MotorType.kBrushless);
  private final DigitalInput processorSensor = new DigitalInput(0); // change later

  private final StatusSignal<Voltage> armVoltage = armMotor.getMotorVoltage();
  private final StatusSignal<Current> armCurrent = armMotor.getStatorCurrent();
  private final StatusSignal<Angle> armPosition = armMotor.getPosition();
  private final StatusSignal<AngularVelocity> armVelocity = armMotor.getVelocity();

  // Updated in updateInputs not sure what to do
  // private final double intakeVoltage = intakeMotor.getAppliedOutput();
  // private final double intakeCurrent = intakeMotor.getOutputCurrent();
  // private final double intakePosition = intakeMotor.getEncoder().getPosition();
  // private final double intakeVelocity = intakeMotor.getEncoder().getVelocity();

  private final VoltageOut armVoltageRequest = new VoltageOut(0).withEnableFOC(true);
  private double intakeVoltageRequest = 0;
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
        20.0, armVelocity, armVoltage, armCurrent, armPosition);
    armMotor.optimizeBusUtilization();

    // Register signals to be updated
    PhoenixUtil.registerSignals(false, armVelocity, armVoltage, armCurrent, armPosition);
  }

  @Override
  public void updateInputs(ProcessorIOInputs inputs) {
    armMotor.setControl(armVoltageRequest);
    intakeMotor.setVoltage(intakeVoltageRequest);
    inputs.armAppliedVolts = armVoltage.getValueAsDouble();
    inputs.armCurrentAmps = armCurrent.getValueAsDouble();
    inputs.armPositionRotations = armPosition.getValueAsDouble();
    inputs.armAngularVelocityRotationsPerMinute = armVelocity.getValueAsDouble();
    inputs.intakeAppliedVolts = intakeMotor.getAppliedOutput();
    inputs.intakeCurrentAmps = intakeMotor.getOutputCurrent();
    inputs.intakePositionRotations = intakeMotor.getEncoder().getPosition();
    inputs.intakeAngularVelocityRotationsPerMinute = intakeMotor.getEncoder().getVelocity();
    inputs.processorSensor = processorSensor.get();
  }

  @Override
  public void armSetVoltage(double volts) {
    armVoltageRequest.withOutput(volts);
  }

  @Override
  public void intakeSetVoltage(double volts) {
    intakeVoltageRequest = volts;
  }
}
