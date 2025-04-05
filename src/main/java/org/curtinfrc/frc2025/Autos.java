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
    var trajectory = routine.trajectory("startToI");
    routine.active().onTrue(trajectory.cmd());
    trajectory
        .done()
        .onTrue(
            drive
                .autoAlign(() -> trajectory.getFinalPose().get())
                .until(drive.atSetpoint.and(elevator.atSetpoint))
                .andThen(ejector.eject(20))
                .until(ejector.backSensor.negate()));
    trajectory
        .atTime("RaiseElevator")
        .onTrue(
            elevator
                .goToSetpoint(ElevatorSetpoints.L2, intake.backSensor.negate())
                .until(ejector.backSensor.negate()));
    return routine;
  }

  public static AutoRoutine twoPieceLeft(
      AutoFactory factory, Drive drive, Ejector ejector, Elevator elevator, Intake intake) {
    var routine = factory.newRoutine("twoPieceLeft");
    var startToFirst = routine.trajectory("startToI");
    var firstToHP = routine.trajectory("iToHp");
    var hpToSecond = routine.trajectory("hpToL");

    routine.active().onTrue(startToFirst.cmd());

    startToFirst
        .done()
        .onTrue(
            drive
                .autoAlign(() -> startToFirst.getFinalPose().get())
                .until(drive.atSetpoint.and(elevator.atSetpoint))
                .andThen(ejector.eject(20))
                .until(ejector.backSensor.negate())
                .andThen(firstToHP.cmd())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    startToFirst
        .atTime("RaiseElevator")
        .onTrue(
            elevator
                .goToSetpoint(ElevatorSetpoints.L2, intake.backSensor.negate())
                .until(ejector.backSensor.negate()));

    firstToHP
        .done()
        .onTrue(
            drive
                .autoAlign(() -> firstToHP.getFinalPose().get())
                .until(drive.atSetpoint)
                .andThen(waitUntil(intake.frontSensor).andThen(hpToSecond.cmd()))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    hpToSecond
        .done()
        .onTrue(
            drive
                .autoAlign(() -> hpToSecond.getFinalPose().get())
                .until(drive.atSetpoint.and(elevator.atSetpoint))
                .andThen(ejector.eject(20))
                .until(ejector.backSensor.negate())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    hpToSecond
        .atTime("RaiseElevator")
        .onTrue(
            elevator
                .goToSetpoint(ElevatorSetpoints.L3, intake.backSensor.negate())
                .until(ejector.backSensor.negate()));

    return routine;
  }

  public static AutoRoutine threePieceLeft(
      AutoFactory factory, Drive drive, Ejector ejector, Elevator elevator, Intake intake) {
    var routine = factory.newRoutine("twoPieceLeft");
    var startToFirst = routine.trajectory("startToI");
    var firstToHP = routine.trajectory("iToHp");
    var hpToSecond = routine.trajectory("hpToL");
    var secondToHp = routine.trajectory("lToHp");
    var hpToThird = routine.trajectory("hpToK");

    routine.active().onTrue(startToFirst.cmd());

    startToFirst
        .done()
        .onTrue(
            drive
                .autoAlign(() -> startToFirst.getFinalPose().get())
                .until(drive.atSetpoint.and(elevator.atSetpoint))
                .andThen(ejector.eject(20))
                .until(ejector.backSensor.negate())
                .andThen(firstToHP.cmd())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    startToFirst
        .atTime("RaiseElevator")
        .onTrue(
            elevator
                .goToSetpoint(ElevatorSetpoints.L2, intake.backSensor.negate())
                .until(ejector.backSensor.negate()));

    firstToHP
        .done()
        .onTrue(
            drive
                .autoAlign(() -> firstToHP.getFinalPose().get())
                .until(drive.atSetpoint.and(intake.frontSensor.or(ejector.backSensor)))
                .andThen(hpToSecond.cmd())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    hpToSecond
        .done()
        .onTrue(
            drive
                .autoAlign(() -> hpToSecond.getFinalPose().get())
                .until(drive.atSetpoint)
                .andThen(ejector.eject(20))
                .until(ejector.backSensor.negate())
                .andThen(secondToHp.cmd())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    secondToHp
        .done()
        .onTrue(
            drive
                .autoAlign(() -> secondToHp.getFinalPose().get())
                .until(drive.atSetpoint.and(intake.frontSensor.or(ejector.backSensor)))
                .andThen(hpToThird.cmd())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    hpToSecond
        .atTime("RaiseElevator")
        .onTrue(
            elevator
                .goToSetpoint(ElevatorSetpoints.L3, intake.backSensor.negate())
                .until(ejector.backSensor.negate()));

    hpToThird
        .done()
        .onTrue(
            sequence(
                    drive
                        .autoAlign(() -> hpToThird.getFinalPose().get())
                        .until(drive.atSetpoint.and(elevator.atSetpoint)),
                    ejector.eject(20).until(ejector.backSensor.negate()),
                    parallel(
                        drive.autoAlign(DriveSetpoints.CLOSE_LEFT::getPose),
                        elevator
                            .goToSetpoint(ElevatorSetpoints.AlgaePopLow, intake.backSensor.negate())
                            .asProxy(),
                        ejector.eject(30)))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    hpToThird
        .atTime("RaiseElevator")
        .onTrue(
            elevator
                .goToSetpoint(ElevatorSetpoints.L3, intake.backSensor.negate())
                .until(ejector.backSensor.negate()));

    return routine;
  }

  public static AutoRoutine fourPieceLeft(
      AutoFactory factory, Drive drive, Ejector ejector, Elevator elevator, Intake intake) {
    var routine = factory.newRoutine("twoPieceLeft");
    var startToI = routine.trajectory("startToI");
    var jToHP = routine.trajectory("jToHp");
    var hpToJ = routine.trajectory("hpToJ");
    var iToHp = routine.trajectory("iToHp");
    var hpToL = routine.trajectory("hpToL");
    var lToHp = routine.trajectory("lToHp");
    var hpToK = routine.trajectory("hpToK");

    routine.active().onTrue(startToI.cmd());

    startToI
        .done()
        .onTrue(
            drive
                .autoAlign(() -> startToI.getFinalPose().get())
                .until(drive.atSetpoint.and(elevator.atSetpoint))
                .andThen(ejector.eject(20))
                .until(ejector.backSensor.negate())
                .andThen(iToHp.cmd())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    startToI
        .atTime("RaiseElevator")
        .onTrue(
            elevator
                .goToSetpoint(ElevatorSetpoints.L2, intake.backSensor.negate())
                .until(ejector.backSensor.negate()));

    iToHp
        .done()
        .onTrue(
            drive
                .autoAlign(() -> iToHp.getFinalPose().get())
                .until(drive.atSetpoint.and(intake.frontSensor.or(ejector.backSensor)))
                .andThen(hpToJ.cmd())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    hpToJ
        .done()
        .onTrue(
            drive
                .autoAlign(() -> hpToJ.getFinalPose().get())
                .until(drive.atSetpoint.and(elevator.atSetpoint))
                .andThen(ejector.eject(20))
                .until(ejector.backSensor.negate())
                .andThen(jToHP.cmd())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    jToHP
        .done()
        .onTrue(
            drive
                .autoAlign(() -> jToHP.getFinalPose().get())
                .until(drive.atSetpoint)
                .andThen(waitUntil(intake.frontSensor).andThen(hpToL.cmd()))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    hpToJ
        .atTime("RaiseElevator")
        .onTrue(
            elevator
                .goToSetpoint(ElevatorSetpoints.L3, intake.backSensor.negate())
                .until(ejector.backSensor.negate()));

    hpToL
        .done()
        .onTrue(
            drive
                .autoAlign(() -> hpToL.getFinalPose().get())
                .until(drive.atSetpoint.and(elevator.atSetpoint))
                .andThen(ejector.eject(20))
                .until(ejector.backSensor.negate())
                .andThen(lToHp.cmd())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    lToHp
        .done()
        .onTrue(
            drive
                .autoAlign(() -> lToHp.getFinalPose().get())
                .until(drive.atSetpoint.and(intake.frontSensor.or(ejector.backSensor)))
                .andThen(hpToK.cmd())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    hpToL
        .atTime("RaiseElevator")
        .onTrue(
            elevator
                .goToSetpoint(ElevatorSetpoints.L3, intake.backSensor.negate())
                .until(ejector.backSensor.negate()));

    hpToK
        .done()
        .onTrue(
            drive
                .autoAlign(() -> hpToK.getFinalPose().get())
                .until(drive.atSetpoint.and(elevator.atSetpoint))
                .andThen(ejector.eject(20))
                .until(ejector.backSensor.negate())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    hpToK
        .atTime("RaiseElevator")
        .onTrue(
            elevator
                .goToSetpoint(ElevatorSetpoints.L3, intake.backSensor.negate())
                .until(ejector.backSensor.negate()));

    return routine;
  }
}
