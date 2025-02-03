package org.curtinfrc.frc2025;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

public class Autos {
  private final AutoFactory factory;

  public Autos(AutoFactory factory) {
    this.factory = factory;
  }

  public AutoRoutine followPath(String path) {
    AutoRoutine routine = factory.newRoutine("followPath" + path);

    AutoTrajectory trajectory = routine.trajectory(path);

    routine.active().onTrue(trajectory.cmd());

    return routine;
  }

  public AutoRoutine jkab() {
    AutoRoutine routine = factory.newRoutine("jkab");
    AutoTrajectory CloseLeftHP_J = routine.trajectory("CloseLeftHP_J");
    AutoTrajectory CloseLeftHP_K = routine.trajectory("CloseLeftHP_K");
    AutoTrajectory J_CloseLeftHP = routine.trajectory("J_CloseLeftHP");
    AutoTrajectory K_CloseLeftHP = routine.trajectory("K_CloseLeftHP");
    AutoTrajectory CloseLeftHP_A = routine.trajectory("CloseLeftHP_A");
    AutoTrajectory CloseLeftHP_B = routine.trajectory("CloseLeftHP_B");
    AutoTrajectory A_CloseLeftHP = routine.trajectory("A_CloseLeftHP");
    AutoTrajectory B_CloseLeftHP = routine.trajectory("B_CloseLeftHP");

    routine.active().onTrue(CloseLeftHP_J.cmd());
    CloseLeftHP_J.done().onTrue(J_CloseLeftHP.cmd());
    J_CloseLeftHP.done().onTrue(CloseLeftHP_K.cmd());
    CloseLeftHP_K.done().onTrue(K_CloseLeftHP.cmd());
    K_CloseLeftHP.done().onTrue(CloseLeftHP_A.cmd());
    CloseLeftHP_A.done().onTrue(A_CloseLeftHP.cmd());
    A_CloseLeftHP.done().onTrue(CloseLeftHP_B.cmd());
    CloseLeftHP_B.done().onTrue(B_CloseLeftHP.cmd());

    return routine;
  }
}
