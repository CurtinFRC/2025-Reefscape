package org.curtinfrc.frc2025.subsystems.elevator;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;

public class ElevatorIONeoMaxMotionLaserCAN extends ElevatorIONeoMaxMotionDistance {
  private LaserCan lc;
  private double distance = -1;

  public ElevatorIONeoMaxMotionLaserCAN() {
    lc = new LaserCan(ElevatorConstants.distanceSensorPort);
    try {
      lc.setRangingMode(LaserCan.RangingMode.SHORT);
      lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
  }

  public void reset() {
    LaserCan.Measurement measurement = lc.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      distance = measurement.distance_mm;
    }

    if (distance >= 0) {
      elevatorEncoder.setPosition(ElevatorIONeoMaxMotion.convertSetpoint(distance));
    }
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.point = this.setpoint;
    inputs.distanceSensorReading = distance;
    inputs.encoderReading = elevatorEncoder.getPosition();
  }
}
