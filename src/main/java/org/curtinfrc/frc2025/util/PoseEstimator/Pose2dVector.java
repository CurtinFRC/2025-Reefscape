package org.curtinfrc.frc2025.util.PoseEstimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class Pose2dVector extends Pose2d {
  private Double x, y;
  private Rotation2d rotation;

  Double stdDev;
  Double angularStdDev;

  public Pose2dVector(Double newX, Double newY, Rotation2d newRotation) {
    x = newX;
    y = newY;
    rotation = newRotation;

    stdDev = 1000.0;
    angularStdDev = 1000.0;
  }

  public Pose2dVector(
      Double newX, Double newY, Rotation2d newRotation, Double newStdDev, Double newAngularStdDev) {
    x = newX;
    y = newY;
    rotation = newRotation;

    stdDev = newStdDev;
    angularStdDev = newAngularStdDev;
  }

  public Pose2dVector(Pose2d newPose) {
    x = newPose.getX();
    y = newPose.getY();
    rotation = newPose.getRotation();
  }

  public Pose2dVector() {
    x = 0.;
    y = 0.;
    rotation = new Rotation2d();
  }

  public Pose2dVector withStdDevs(Double newStdDev, Double newAngularStdDev) {
    return new Pose2dVector(x, y, rotation, newStdDev, newAngularStdDev);
  }

  public Pose2dVector plus(Pose2dVector other) {
    return new Pose2dVector(x + other.x, y + other.y, rotation.plus(other.rotation))
        .withStdDevs(stdDev + other.stdDev, angularStdDev + other.angularStdDev);
  }

  public Pose2dVector times(Double scalar) {
    return new Pose2dVector(x * scalar, y * scalar, rotation)
        .withStdDevs(stdDev * scalar, angularStdDev);
  }

  public Pose2dVector negate() {
    return new Pose2dVector(x, y, rotation.unaryMinus());
  }

  public Pose2dVector minus(Pose2dVector other) {
    return this.plus(other.negate());
  }

  public Pose2dVector divide(Double scalar) {
    Double scalarInverse = 1. / scalar;
    return this.times(scalarInverse);
  }

  public Pose2dVector merge(Pose2dVector with) {
    Double confidence = 1 / stdDev;
    Double withConfidence = 1 / with.stdDev;

    Double angularCofidence = 1 / angularStdDev;
    Double angularWithConfidence = 1 / with.angularStdDev;
    return new Pose2dVector(
        x * confidence + with.x * withConfidence / (confidence + withConfidence),
        x * confidence + with.x * withConfidence / (confidence + withConfidence),
        rotation
            .times(confidence)
            .plus(with.rotation.times(angularWithConfidence))
            .div(angularCofidence + angularWithConfidence),
        stdDev + with.stdDev,
        angularStdDev + with.angularStdDev);
  }
}
