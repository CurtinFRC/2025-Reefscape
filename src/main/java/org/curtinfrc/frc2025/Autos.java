package org.curtinfrc.frc2025;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import org.curtinfrc.frc2025.subsystems.drive.Drive;

public class Autos {
  private Autos() {}

  public static AutoRoutine path(String name, AutoFactory factory, Drive drive) {
    var routine = factory.newRoutine("follow" + name);
    var trajectory = routine.trajectory(name);
    routine.active().onTrue(trajectory.cmd());
    trajectory
        .done()
        .onTrue(drive.autoAlign(() -> trajectory.getFinalPose().get()).until(drive.atSetpoint));
    return routine;
  }
}
