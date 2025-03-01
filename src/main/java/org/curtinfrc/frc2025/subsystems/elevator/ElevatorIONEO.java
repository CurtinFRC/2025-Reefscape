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

public class ElevatorIONEO implements ElevatorIO {
  private final SparkMax elevatorLeader = new SparkMax(leaderPort, MotorType.kBrushless);
  private final SparkMax elevatorFollower = new SparkMax(followerPort, MotorType.kBrushless);

  private final RelativeEncoder elevatorEncoder = elevatorLeader.getEncoder();
  private DigitalInput touch = new DigitalInput(resetPort);

  public ElevatorIONEO() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(0, currentLimit).idleMode(IdleMode.kCoast).inverted(false);

    SparkUtil.tryUntilOk(
        5,
        () ->
            elevatorLeader.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    config.follow(leaderPort);

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
    inputs.hominSensor = false;
  }

  @Override
  public void setVoltage(double volts) {
    elevatorLeader.setVoltage(volts);
  }

  @Override
  public void zero() {
    elevatorEncoder.setPosition(0);
  }
}
