package org.curtinfrc.frc2025.subsystems.drive;

import static org.curtinfrc.frc2025.Constants.FIELD_LENGTH;
import static org.curtinfrc.frc2025.Constants.FIELD_WIDTH;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.ArrayList;
import java.util.List;
import org.curtinfrc.frc2025.Constants;
import org.curtinfrc.frc2025.util.Force;
import org.littletonrobotics.junction.Logger;

public class RepulsorFieldPlanner {
  abstract static class Obstacle {
    double strength = 1.0;
    boolean positive = true;

    public Obstacle(double strength, boolean positive) {
      this.strength = strength;
      this.positive = positive;
    }

    public abstract Force getForceAtPosition(Translation2d position, Translation2d target);

    protected double distToForceMag(double dist) {
      var forceMag = strength / (0.00001 + Math.abs(dist * dist));
      forceMag *= positive ? 1 : -1;
      return forceMag;
    }

    protected double distToForceMag(double dist, double falloff) {
      var original = strength / (0.00001 + Math.abs(dist * dist));
      var falloffMag = strength / (0.00001 + Math.abs(falloff * falloff));
      return Math.max(original - falloffMag, 0) * (positive ? 1 : -1);
    }
  }

  static class PointObstacle extends Obstacle {
    Translation2d loc;
    double radius = 0.5;

    public PointObstacle(Translation2d loc, double strength, boolean positive) {
      super(strength, positive);
      this.loc = loc;
    }

    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      var dist = loc.getDistance(position);
      if (dist > 4) {
        return new Force();
      }
      var outwardsMag = distToForceMag(loc.getDistance(position) - radius);
      var initial = new Force(outwardsMag, position.minus(loc).getAngle());
      var theta = target.minus(position).getAngle().minus(position.minus(loc).getAngle());
      double mag = outwardsMag * Math.signum(Math.sin(theta.getRadians() / 2)) / 2;

      return initial
          .rotateBy(Rotation2d.kCCW_90deg)
          .div(initial.getNorm())
          .times(mag)
          .plus(initial);
    }
  }

  static class SnowmanObstacle extends Obstacle {
    Translation2d loc;
    double radius = 0.5;

    public SnowmanObstacle(Translation2d loc, double strength, double radius, boolean positive) {
      super(strength, positive);
      this.loc = loc;
      this.radius = radius;
    }

    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      var targetToLoc = loc.minus(target);
      var targetToLocAngle = targetToLoc.getAngle();
      // 1 meter away from loc, opposite target.
      var sidewaysCircle = new Translation2d(1, targetToLoc.getAngle()).plus(loc);
      var sidewaysMag = distToForceMag(sidewaysCircle.getDistance(position));
      var outwardsMag = distToForceMag(Math.max(0.01, loc.getDistance(position) - radius));
      var initial = new Force(outwardsMag, position.minus(loc).getAngle());

      // flip the sidewaysMag based on which side of the goal-sideways circle the robot is on
      var sidewaysTheta =
          target.minus(position).getAngle().minus(position.minus(sidewaysCircle).getAngle());

      double sideways = sidewaysMag * Math.signum(Math.sin(sidewaysTheta.getRadians()));
      var sidewaysAngle = targetToLocAngle.rotateBy(Rotation2d.kCCW_90deg);
      return new Force(sideways, sidewaysAngle).plus(initial);
    }
  }

  static class HorizontalObstacle extends Obstacle {
    double y;

    public HorizontalObstacle(double y, double strength, boolean positive) {
      super(strength, positive);
      this.y = y;
    }

    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      return new Force(0, distToForceMag(y - position.getY(), 1));
    }
  }

  static class VerticalObstacle extends Obstacle {
    double x;

    public VerticalObstacle(double x, double strength, boolean positive) {
      super(strength, positive);
      this.x = x;
    }

    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      return new Force(distToForceMag(x - position.getX(), 1), 0);
    }
  }

  private static class TeardropObstacle extends Obstacle {
    final Translation2d loc;
    final double primaryMaxRange;
    final double primaryRadius;
    final double tailStrength;
    final double tailLength;

    public TeardropObstacle(
        Translation2d loc,
        double primaryStrength,
        double primaryMaxRange,
        double primaryRadius,
        double tailStrength,
        double tailLength) {
      super(primaryStrength, true);
      this.loc = loc;
      this.primaryMaxRange = primaryMaxRange;
      this.primaryRadius = primaryRadius;
      this.tailStrength = tailStrength;
      this.tailLength = tailLength + primaryMaxRange;
    }

    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      var targetToLoc = loc.minus(target);
      var targetToLocAngle = targetToLoc.getAngle();
      var sidewaysPoint = new Translation2d(tailLength, targetToLoc.getAngle()).plus(loc);

      var positionToLocation = position.minus(loc);
      var positionToLocationDistance = positionToLocation.getNorm();
      Translation2d outwardsForce;
      if (positionToLocationDistance <= primaryMaxRange) {
        outwardsForce =
            new Translation2d(
                distToForceMag(
                    Math.max(positionToLocationDistance - primaryRadius, 0),
                    primaryMaxRange - primaryRadius),
                positionToLocation.getAngle());
      } else {
        outwardsForce = Translation2d.kZero;
      }

      var positionToLine = position.minus(loc).rotateBy(targetToLocAngle.unaryMinus());
      var distanceAlongLine = positionToLine.getX();

      Translation2d sidewaysForce;
      var distanceScalar = distanceAlongLine / tailLength;
      if (distanceScalar >= 0 && distanceScalar <= 1) {
        var secondaryMaxRange =
            MathUtil.interpolate(primaryMaxRange, 0, distanceScalar * distanceScalar);
        var distanceToLine = Math.abs(positionToLine.getY());
        if (distanceToLine <= secondaryMaxRange) {
          double strength;
          if (distanceAlongLine < primaryMaxRange) {
            strength = tailStrength * (distanceAlongLine / primaryMaxRange);
          } else {
            strength =
                -tailStrength * distanceAlongLine / (tailLength - primaryMaxRange)
                    + tailLength * tailStrength / (tailLength - primaryMaxRange);
          }
          strength *= 1 - distanceToLine / secondaryMaxRange;

          var sidewaysMag = tailStrength * strength * (secondaryMaxRange - distanceToLine);
          // flip the sidewaysMag based on which side of the goal-sideways circle the robot is on
          var sidewaysTheta =
              target.minus(position).getAngle().minus(position.minus(sidewaysPoint).getAngle());
          sidewaysForce =
              new Translation2d(
                  sidewaysMag * Math.signum(Math.sin(sidewaysTheta.getRadians())),
                  targetToLocAngle.rotateBy(Rotation2d.kCCW_90deg));
        } else {
          sidewaysForce = Translation2d.kZero;
        }
      } else {
        sidewaysForce = Translation2d.kZero;
      }

      return new Force(
          outwardsForce.plus(sidewaysForce).getNorm(),
          outwardsForce.plus(sidewaysForce).getAngle());
    }
  }

  public static final double GOAL_STRENGTH = 1.2;

  public static final List<Obstacle> FIELD_OBSTACLES =
      List.of(
          new TeardropObstacle(new Translation2d(4.49, 4), 1.2, 2.2, 1.03, 3, 2),
          new TeardropObstacle(new Translation2d(13.08, 4), 1.2, 2.2, 1.03, 3, 2));

  public static final List<Obstacle> WALLS =
      List.of(
          new HorizontalObstacle(0.0, 2, true),
          new HorizontalObstacle(FIELD_WIDTH, 1.4, false),
          new VerticalObstacle(0.0, 2, true),
          new VerticalObstacle(Constants.FIELD_LENGTH, 1.4, false));

  private List<Obstacle> fixedObstacles = new ArrayList<>();
  private Translation2d goal = Translation2d.kZero;

  private static final int ARROWS_X = RobotBase.isSimulation() ? 40 : 0;
  private static final int ARROWS_Y = RobotBase.isSimulation() ? 20 : 0;
  private static final int ARROWS_SIZE = (ARROWS_X + 1) * (ARROWS_Y + 1);

  private ArrayList<Pose2d> arrows = new ArrayList<>(ARROWS_SIZE);

  public RepulsorFieldPlanner() {
    fixedObstacles.addAll(FIELD_OBSTACLES);
    fixedObstacles.addAll(WALLS);
    for (int i = 0; i < ARROWS_SIZE; i++) {
      arrows.add(new Pose2d());
    }
  }

  private Pose2d arrowBackstage = new Pose2d(-10, -10, Rotation2d.kZero);

  // A grid of arrows drawn in AScope
  void updateArrows() {
    for (int x = 0; x <= ARROWS_X; x++) {
      for (int y = 0; y <= ARROWS_Y; y++) {
        var translation =
            new Translation2d(x * FIELD_LENGTH / ARROWS_X, y * FIELD_WIDTH / ARROWS_Y);
        var force = Force.kZero;
        force = force.plus(getObstacleForce(translation, goal));
        force = force.plus(getWallForce(translation, goal));
        force = force.plus(getGoalForce(translation, goal));
        if (force.getNorm() < 1e-6) {
          arrows.set(x * (ARROWS_Y + 1) + y, arrowBackstage);
        } else {
          var rotation = force.getAngle();

          arrows.set(x * (ARROWS_Y + 1) + y, new Pose2d(translation, rotation));
        }
      }
    }

    if (RobotBase.isSimulation()) {
      Pose2d[] arr = new Pose2d[0];
      Logger.recordOutput("Repulsor/arrows", arrows.toArray(arr));
    }
  }

  Force getGoalForce(Translation2d curLocation, Translation2d goal) {
    var displacement = goal.minus(curLocation);
    if (displacement.getNorm() == 0) {
      return new Force();
    }
    var direction = displacement.getAngle();
    var mag =
        GOAL_STRENGTH * (1 + 1.0 / (0.0001 + displacement.getNorm() * displacement.getNorm()));
    return new Force(mag, direction);
  }

  Force getWallForce(Translation2d curLocation, Translation2d target) {
    var force = Force.kZero;
    for (Obstacle obs : WALLS) {
      force = force.plus(obs.getForceAtPosition(curLocation, target));
    }
    return force;
  }

  Force getObstacleForce(Translation2d curLocation, Translation2d target) {
    var force = Force.kZero;
    for (Obstacle obs : FIELD_OBSTACLES) {
      force = force.plus(obs.getForceAtPosition(curLocation, target));
    }
    return force;
  }

  Force getForce(Translation2d curLocation, Translation2d target) {
    var goalForce =
        getGoalForce(curLocation, target)
            .plus(getObstacleForce(curLocation, target))
            .plus(getWallForce(curLocation, target))
            .times(Math.min(1.0, curLocation.getDistance(target))); // Adjust scaling
    return goalForce;
  }

  public static record RepulsorSample(Translation2d goal, double vx, double vy) {}

  public void setGoal(Translation2d goal) {
    this.goal = goal;
    updateArrows();
  }

  public RepulsorSample calculate(Pose2d pose, ChassisSpeeds currentSpeeds, double maxSpeed) {
    double stepSize_m = 0.02;
    var curTrans = pose.getTranslation();
    var err = curTrans.minus(goal);

    Logger.recordOutput("Repulsor/err", curTrans.getDistance(goal));
    Logger.recordOutput("Repulsor/toggle_dist", stepSize_m * 1.5);

    if (err.getNorm() < stepSize_m) {
      return new RepulsorSample(pose.getTranslation(), 0, 0);
    } else {
      var obstacleForce = getObstacleForce(curTrans, goal).plus(getWallForce(curTrans, goal));
      var netForce = obstacleForce;
      netForce = getGoalForce(curTrans, goal).plus(netForce);
      var dist = err.getNorm();
      stepSize_m = Math.min(5.14, Math.sqrt(6 /* 14 */ * dist)) * 0.02;
      var step = new Translation2d(stepSize_m, netForce.getAngle());
      return new RepulsorSample(goal, (step.getX() / 0.02), (step.getY() / 0.02));
    }
  }
}
