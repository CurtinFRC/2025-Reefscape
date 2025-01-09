package org.curtinfrc.frc2025.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ElevatorIOSim implements ElevatorIO {
  private DCMotor motor = DCMotor.getNEO(ElevatorConstants.motorCount);
  private DCMotorSim elevatorSim;
  private ElevatorConstants.Setpoints set = ElevatorConstants.Setpoints.NONE;

  public ElevatorIOSim() {
    elevatorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, 0.025, 4.0), motor);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    elevatorSim.update(0.02);

    inputs.point = set;
    inputs.distanceSensorReading = 0;
    inputs.encoderReading = elevatorSim.getAngularPositionRotations();
  }

  public void goToSetpoint(ElevatorConstants.Setpoints point) {
    set = point;

    elevatorSim.setAngle(ElevatorIONeoMaxMotion.convertSetpoint(point.setpoint));
  }
}
