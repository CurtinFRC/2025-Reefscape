package org.curtinfrc.frc2025;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import org.curtinfrc.frc2025.subsystems.drive.Drive;
import org.curtinfrc.frc2025.subsystems.drive.DriveConstants.DriveSetpoints;
import org.curtinfrc.frc2025.subsystems.ejector.Ejector;
import org.curtinfrc.frc2025.subsystems.elevator.Elevator;
import org.curtinfrc.frc2025.subsystems.elevator.ElevatorConstants.ElevatorSetpoints;
import org.curtinfrc.frc2025.subsystems.intake.Intake;

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

  public static AutoRoutine onePieceCentre(
      AutoFactory factory, Drive drive, Ejector ejector, Elevator elevator, Intake intake) {
    var routine = factory.newRoutine("onePieceCentre");
    var trajectory = routine.trajectory("onePieceCentre");
    routine.active().onTrue(trajectory.cmd());
    trajectory
        .done()
        .onTrue(
            drive
                .autoAlign(() -> trajectory.getFinalPose().get())
                .until(drive.atSetpoint)
                .andThen(
                    elevator
                        .goToSetpoint(ElevatorSetpoints.L3, intake.backSensor.negate())
                        .until(elevator.atSetpoint)
                        .andThen(
                            parallel(ejector.eject(15)),
                            elevator.goToSetpoint(ElevatorSetpoints.L3, intake.backSensor.negate()))
                        .until(ejector.backSensor.negate()))
                .andThen(
                    elevator
                        .goToSetpoint(ElevatorSetpoints.BASE, intake.backSensor.negate())
                        .until(elevator.atSetpoint))
                .andThen(
                    parallel(drive.autoAlign(() -> DriveSetpoints.FAR.getPose()), ejector.eject(30))
                        .withTimeout(1))
                .withName("autoScore")
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    return routine;
  }

  public static AutoRoutine onePieceLeft(
      AutoFactory factory, Drive drive, Ejector ejector, Elevator elevator, Intake intake) {
    var routine = factory.newRoutine("onePieceLeft");
    var trajectory = routine.trajectory("onePieceLeft");
    routine.active().onTrue(trajectory.cmd());
    trajectory
        .done()
        .onTrue(
            drive
                .autoAlign(() -> trajectory.getFinalPose().get())
                .until(drive.atSetpoint)
                .andThen(ejector.eject(20))
                .until(ejector.backSensor.negate()));
    trajectory
        .atTime("FirstElevatorRaise")
        .onTrue(elevator.goToSetpoint(ElevatorSetpoints.L2, intake.backSensor.negate()));
    return routine;
  }

  public static AutoRoutine twoPieceLeft(
      AutoFactory factory, Drive drive, Ejector ejector, Elevator elevator, Intake intake) {
    var routine = factory.newRoutine("onePieceLeft");
    var startToFirst = routine.trajectory("onePieceLeft");
    var firstToHP = routine.trajectory("firstToHp");
    var hpToSecond = routine.trajectory("hpToSecond");

    routine.active().onTrue(startToFirst.cmd());

    startToFirst
        .done()
        .onTrue(
            drive
                .autoAlign(() -> startToFirst.getFinalPose().get())
                .until(drive.atSetpoint)
                .andThen(ejector.eject(20))
                .until(ejector.backSensor.negate())
                .andThen(firstToHP.cmd())
                .withName("StartToFirstAlign"));

    startToFirst
        .atTime("RaiseElevator")
        .onTrue(elevator.goToSetpoint(ElevatorSetpoints.L2, intake.backSensor.negate()));

    firstToHP
        .done()
        .onTrue(
            drive
                .autoAlign(() -> firstToHP.getFinalPose().get())
                .until(drive.atSetpoint)
                .andThen(waitUntil(intake.frontSensor).andThen(hpToSecond.cmd())));

    return routine;
  }
}
