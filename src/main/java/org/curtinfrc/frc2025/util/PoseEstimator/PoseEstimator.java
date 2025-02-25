package org.curtinfrc.frc2025.util.PoseEstimator;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public final class PoseEstimator {
  Measurement current;

  ArrayList<Measurement> newMeasurements = new ArrayList<Measurement>();

  public PoseEstimator(Pose2d initialPose) {
    current = new Measurement(initialPose);
  }

  public Pose2d getPose() {
    return current.getPoseAsPose2d();
  }

  public void update() {
    current.log("Pose/Current/");
    for (int i = 0; i < newMeasurements.size(); i++) {
      newMeasurements.get(i).log("Pose/Measurements/" + i + "/");
    }
    Logger.recordOutput("Pose/Measurements/Size", newMeasurements.size());
    while (newMeasurements.size() != 0) {

      current.physicsUpdate(newMeasurements.get(0).time());

      current = current.merge(newMeasurements.get(0));

      newMeasurements.remove(0);
    }
  }

  public void addMeasurement(Measurement newMeasurement) {

    boolean measurementAdded = false;
    if (newMeasurements.size() == 0) {
      newMeasurements.add(newMeasurement);
      measurementAdded = true;
    } else {
      Double newMeasurementTime = newMeasurement.time();
      for (int i = 0; i < newMeasurements.size(); i++) {
        Double checkMeasurementTime = newMeasurements.get(i).time();
        if (Math.abs(checkMeasurementTime - newMeasurementTime) < 0.001) {
          newMeasurements.add(i, newMeasurement);
          measurementAdded = true;
        } else if (checkMeasurementTime > newMeasurementTime) {

          newMeasurements.add(i, newMeasurement);
          measurementAdded = true;
        }
      }

      if (measurementAdded == false) {

        newMeasurements.add(newMeasurement);
        measurementAdded = true;
      }
    }
  }
}
