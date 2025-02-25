package org.curtinfrc.frc2025.util.PoseEstimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;

public final class Pose2dVector extends Pose2d {
  private Double x, y;
  private Rotation2d rotation;

  Double stdDev = 1000.;
  Double angularStdDev = 1000.;

  public Pose2dVector(Double newX, Double newY, Rotation2d newRotation) {
    x = newX;
    y = newY;
    rotation = newRotation;

    // Logger.recordOutput("Pose/stdDev", stdDev);
  }

  public Pose2dVector(
      Double newX, Double newY, Rotation2d newRotation, Double newStdDev, Double newAngularStdDev) {
    x = newX;
    y = newY;
    rotation = newRotation;

    stdDev = newStdDev;
    angularStdDev = newAngularStdDev;
    // Logger.recordOutput("Pose/stdDev", stdDev);
  }

  public Pose2dVector(Pose2d newPose) {
    x = newPose.getX();
    y = newPose.getY();
    rotation = newPose.getRotation();
    // Logger.recordOutput("Pose/stdDev", stdDev);
  }

  public Pose2dVector() {
    x = 0.;
    y = 0.;
    rotation = new Rotation2d();
    // Logger.recordOutput("Pose/stdDev", stdDev);
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
    return new Pose2dVector(
      with.stdDev > 1 ? x : with.x,
      with.stdDev > 1 ? y : with.y, 
      with.angularStdDev > 1 ? rotation : with.rotation, .1, .1);
  }

  public Pose2d asPose2d() {
    return new Pose2d(x, y, rotation);
  }

  public void log(String key) {
    Logger.recordOutput(key + "Pose", asPose2d());
    Logger.recordOutput(key + "StdDev", stdDev);
    Logger.recordOutput(key + "AngularStdDev", angularStdDev);
  }
}
