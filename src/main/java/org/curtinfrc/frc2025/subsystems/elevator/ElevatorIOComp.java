package org.curtinfrc.frc2025.subsystems.elevator;

import static org.curtinfrc.frc2025.subsystems.elevator.ElevatorConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import org.curtinfrc.frc2025.util.SparkUtil;

public class ElevatorIOComp implements ElevatorIO {
  private static final double pulleyRadiusMeters = 0.03055;
  private static final double gearing = 8.1818;
  private static final int LEADER_ID = 31;

  private final SparkMax elevatorLeader = new SparkMax(LEADER_ID, MotorType.kBrushless);
  private final SparkMax elevatorFollower = new SparkMax(32, MotorType.kBrushless);

  private final RelativeEncoder elevatorEncoder = elevatorLeader.getEncoder();
  private DigitalInput reset = new DigitalInput(resetPort);

  public ElevatorIOComp() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(0, currentLimit).idleMode(IdleMode.kBrake).inverted(false);

    SparkUtil.tryUntilOk(
        5,
        () ->
            elevatorLeader.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    config.follow(LEADER_ID);

    SparkUtil.tryUntilOk(
        5,
        () ->
            elevatorFollower.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.appliedVolts = elevatorLeader.getBusVoltage() * elevatorLeader.getAppliedOutput();
    inputs.currentAmps = elevatorLeader.getOutputCurrent();
    inputs.positionRotations = elevatorLeader.getEncoder().getPosition();
    inputs.angularVelocityRotationsPerMinute = elevatorLeader.getEncoder().getVelocity();
    inputs.hominSensor = reset.get();
  }

  @Override
  public void setVoltage(double volts) {
    elevatorLeader.setVoltage(volts);
  }

  @Override
  public void zero() {
    elevatorEncoder.setPosition(0);
  }

  @Override
  public double positionRotationsToMetres(double rotations) {
    return rotations * Math.PI * 2 * pulleyRadiusMeters / gearing;
  }

  @Override
  public double positionMetresToRotations(double metres) {
    return metres / (Math.PI * 2 * pulleyRadiusMeters) * gearing;
  }
}
