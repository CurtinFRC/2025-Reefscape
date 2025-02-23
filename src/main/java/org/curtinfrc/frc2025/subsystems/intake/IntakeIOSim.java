package org.curtinfrc.frc2025.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeters;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.curtinfrc.frc2025.Constants;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class IntakeIOSim implements IntakeIO {
  private final DCMotor intakeMotor = DCMotor.getNEO(1);
  private final DCMotorSim intakeMotorSim;
  private final SimDevice frontImpl;
  private final SimBoolean frontSensor;
  private final SimDevice backImpl;
  private final SimBoolean backSensor;
  private final IntakeSimulation intakeSimulation;
  private double volts = 0;

  public IntakeIOSim(AbstractDriveTrainSimulation driveTrain) {
    intakeMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                intakeMotor, IntakeConstants.intakeMoi, IntakeConstants.motorReduction),
            intakeMotor);

    frontImpl = SimDevice.create("IntakeSensorFront", IntakeConstants.intakeFrontSensorPort);
    frontSensor = frontImpl.createBoolean("IsTriggered", Direction.kInput, false);
    backImpl = SimDevice.create("IntakeSensorBack", IntakeConstants.intakeBackSensorPort);
    backSensor = backImpl.createBoolean("IsTriggered", Direction.kInput, false);

    // Create the intake simulation
    intakeSimulation =
        IntakeSimulation.OverTheBumperIntake(
            "Coral", // Type of game piece
            driveTrain, // DriveTrain reference
            Millimeters.of(Constants.ROBOT_X), // Width of intake
            Meters.of(0.2),
            IntakeSide.BACK, // Mounted on the back of the robot
            1 // Capacity
            );
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    intakeMotorSim.setInputVoltage(volts);
    intakeMotorSim.update(0.02);
    inputs.appliedVolts = intakeMotorSim.getInputVoltage();
    inputs.currentAmps = intakeMotorSim.getCurrentDrawAmps();
    inputs.positionRotations = intakeMotorSim.getAngularPositionRotations();
    inputs.angularVelocityRotationsPerMinute = intakeMotorSim.getAngularVelocityRPM();
    inputs.frontSensor = frontSensor.get();
    inputs.backSensor = backSensor.get();
    inputs.simHasNote = intakeSimulation.getGamePiecesAmount() != 0;
  }

  @Override
  public void setVoltage(double volts) {
    this.volts = volts;
    if (volts == 0) {
      intakeSimulation.stopIntake();
    } else {
      intakeSimulation.startIntake();
    }
  }
}
