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
    routine.active().onTrue(factory.resetOdometry(name).andThen(trajectory.cmd()));
    return routine;
  }

  public static AutoRoutine threePieceLeft(
      AutoFactory factory, Drive drive, Ejector ejector, Elevator elevator, Intake intake) {
    var routine = factory.newRoutine("threePieceLeft");
    var startToFirst = routine.trajectory("startToI");
    var firstToHP = routine.trajectory("iToHp");
    var hpToSecond = routine.trajectory("hpToL");
    var secondToHp = routine.trajectory("lToHp");
    var hpToThird = routine.trajectory("hpToK");
    var kToHp = routine.trajectory("kToHp");

    // routine
    //     .active()
    //     .onTrue(drive.autoAlign(startToFirst.getInitialPose()::get).andThen(startToFirst.cmd()));

    // startToFirst
    //     .done()
    // routine
    //     .active()
    //     .onTrue(
    //         drive
    //             .autoAlign(startToFirst.getFinalPose()::get)
    //             .until(drive.atSetpoint)
    //             .andThen(ejector.eject(30).until(ejector.backSensor.negate()))
    //             .andThen(firstToHP.cmd())
    //             .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    routine
        .active()
        .onTrue(
            parallel(
                    drive.autoAlign(startToFirst.getFinalPose()::get),
                    elevator.goToSetpoint(ElevatorSetpoints.L2, intake.backSensor.negate()))
                .until(drive.atSetpoint)
                .andThen(
                    parallel(
                            ejector.eject(30),
                            elevator.goToSetpoint(ElevatorSetpoints.L2, intake.backSensor.negate()))
                        .until(ejector.backSensor.negate()))
                .andThen(
                    race(
                        firstToHP.cmd(),
                        elevator.goToSetpoint(ElevatorSetpoints.BASE, intake.backSensor.negate())))
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
                .autoAlign(firstToHP.getFinalPose()::get)
                .until(intake.frontSensor)
                .andThen(hpToSecond.cmd())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    hpToSecond
        .done()
        .onTrue(
            drive
                .autoAlign(hpToSecond.getFinalPose()::get)
                .until(drive.atSetpoint)
                .andThen(ejector.eject(20))
                .until(ejector.backSensor.negate())
                .andThen(secondToHp.cmd())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    secondToHp
        .done()
        .onTrue(
            drive
                .autoAlign(secondToHp.getFinalPose()::get)
                .until(intake.frontSensor)
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
                        .autoAlign(hpToThird.getFinalPose()::get)
                        .until(drive.atSetpoint)
                        .andThen(ejector.eject(20).until(ejector.backSensor.negate())),
                    parallel(
                            drive.autoAlign(DriveSetpoints.CLOSE_LEFT::getPose),
                            elevator
                                .goToSetpoint(
                                    ElevatorSetpoints.AlgaePopLow, intake.backSensor.negate())
                                .asProxy(),
                            ejector.pop())
                        .withTimeout(1),
                    secondToHp.cmd())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("Score, Pop algae and drive back"));

    hpToThird
        .atTime("RaiseElevator")
        .onTrue(
            elevator
                .goToSetpoint(ElevatorSetpoints.L3, intake.backSensor.negate())
                .until(ejector.backSensor.negate()));

    return routine;
  }
}
