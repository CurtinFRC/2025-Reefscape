package org.curtinfrc.frc2025.util;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.curtinfrc.frc2025.util.RepulsorFieldPlanner.HorizontalObstacle;
import org.curtinfrc.frc2025.util.RepulsorFieldPlanner.Obstacle;
import org.curtinfrc.frc2025.util.RepulsorFieldPlanner.SnowmanObstacle;
import org.curtinfrc.frc2025.util.RepulsorFieldPlanner.VerticalObstacle;

public class RepulsorConstants {
  public static final double GOAL_STRENGTH = 0.1;

  public static final List<Obstacle> FIELD_OBSTACLES =
      List.of(
          new SnowmanObstacle(new Translation2d(4.49, 4), 1, true),
          new SnowmanObstacle(new Translation2d(13.08, 4), 1, true));
  public static final double FIELD_LENGTH = 16.42;
  public static final double FIELD_WIDTH = 8.16;
  public static final List<Obstacle> WALLS =
      List.of(
          new HorizontalObstacle(0.0, 0.5, true),
          new HorizontalObstacle(FIELD_WIDTH, 0.5, false),
          new VerticalObstacle(0.0, 0.5, true),
          new VerticalObstacle(FIELD_LENGTH, 0.5, false));
}
