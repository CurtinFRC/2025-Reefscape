package org.curtinfrc.frc2025.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;

public class Autos {
  private final AutoFactory factory;

  public Autos(AutoFactory factory) {
    this.factory = factory;
  }

  public AutoRoutine followPath(String path) {
    AutoRoutine routine = factory.newRoutine("followPath" + path);

    AutoTrajectory trajectory = routine.trajectory(path);

    routine.active().onTrue(Commands.sequence(trajectory.resetOdometry(), trajectory.cmd()));

    return routine;
  }
}
