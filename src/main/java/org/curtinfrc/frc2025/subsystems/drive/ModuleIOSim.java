package org.curtinfrc.frc2025.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ModuleIOSim extends ModuleIOTalonFX {
  private static final DCMotor DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);
  private static final DCMotor TURN_GEARBOX = DCMotor.getKrakenX60(1);

  private final DCMotorSim driveSim;
  private final DCMotorSim steerSim;

  public ModuleIOSim(SwerveModuleConstants constants) {
    super(constants);
    // Create drive and steer sim models
    driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DRIVE_GEARBOX, constants.DriveInertia, constants.DriveMotorGearRatio),
            DRIVE_GEARBOX);
    steerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                TURN_GEARBOX, constants.SteerInertia, constants.SteerMotorGearRatio),
            TURN_GEARBOX);
  }

  private double driveOutput(double raw) {
    return raw * constants.DriveMotorGearRatio;
  }

  private double driveVoltage(double raw) {
    return (Math.abs(raw) > constants.DriveFrictionVoltage) ? raw : 0;
  }

  private double steerOutput(double raw) {
    return (constants.SteerMotorInverted ? -raw : raw) * constants.SteerMotorGearRatio;
  }

  private double steerVoltage(double raw) {
    return (Math.abs(raw) > constants.SteerFrictionVoltage) ? raw : 0;
  }

  private void updateSimState(double dt) {
    var driveSimState = driveTalon.getSimState();
    driveSim.setInputVoltage(driveVoltage(driveSimState.getMotorVoltage()));
    driveSim.update(dt);
    driveSimState.setRawRotorPosition(driveOutput(driveSim.getAngularPositionRotations()));
    driveSimState.setRotorVelocity(driveOutput(driveSim.getAngularVelocityRPM() / 60));

    var steerSimState = turnTalon.getSimState();
    var encoderSimState = cancoder.getSimState();
    steerSim.setInputVoltage(steerVoltage(steerSimState.getMotorVoltage()));
    steerSim.update(dt);
    encoderSimState.setRawPosition(-steerSim.getAngularPositionRotations());
    steerSimState.setRawRotorPosition(steerOutput(steerSim.getAngularPositionRotations()));
    steerSimState.setRotorVelocity(steerOutput(steerSim.getAngularVelocityRPM() / 60));
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    super.updateInputs(inputs);
    updateSimState(0.02);
  }
}
