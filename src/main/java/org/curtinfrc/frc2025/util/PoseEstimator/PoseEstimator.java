package org.curtinfrc.frc2025.util.PoseEstimator;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.ArrayList;

public final class PoseEstimator {
  Measurement current;

  ArrayList<ArrayList<Measurement>> newMeasurements;

  public PoseEstimator(Pose2d initialPose) {
    current = new Measurement(initialPose);
  }

  public Pose2d getPose() {
    return new Pose2d(current.getX(), current.getY(), current.getRotation());
  }

  public void update() {
    while (newMeasurements.size() != 0) {

      current.physicsUpdate(newMeasurements.get(0).get(0).time());

      while (newMeasurements.get(0).size() != 0) {
        current = current.merge(newMeasurements.get(0).get(0));
        newMeasurements.get(0).remove(0);
      }

      newMeasurements.remove(0);
    }
  }

  public void addMeasurement(Measurement newMeasurement) {

    boolean measurementAdded = false;
    if (newMeasurements.size() == 0) {
      ArrayList<Measurement> timeStep = new ArrayList<Measurement>();
      timeStep.add(newMeasurement);
      newMeasurements.add(timeStep);
      measurementAdded = true;
    } else {
      Double newMeasurementTime = newMeasurement.time();
      for (int i = 0; i < newMeasurements.size(); i++) {
        Double checkMeasurementTime = newMeasurements.get(i).get(0).time();
        if (Math.abs(checkMeasurementTime - newMeasurementTime) < 0.001) {
          newMeasurements.get(i).add(newMeasurement);
          measurementAdded = true;
        } else if (checkMeasurementTime > newMeasurementTime) {
          ArrayList<Measurement> timeStep = new ArrayList<Measurement>();
          timeStep.add(newMeasurement);

          newMeasurements.add(i, timeStep);
          measurementAdded = true;
        }
      }

      if (measurementAdded == false) {
        ArrayList<Measurement> timeStep = new ArrayList<Measurement>();
        timeStep.add(newMeasurement);

        newMeasurements.add(timeStep);
        measurementAdded = true;
      }
    }
  }
}
