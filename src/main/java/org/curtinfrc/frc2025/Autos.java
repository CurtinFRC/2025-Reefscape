package org.curtinfrc.frc2025;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    routine
        .active()
        .onTrue(
            sequence(
                    waitSeconds(1),
                    drive.autoAlign(() -> DriveSetpoints.G.getPose()).until(drive.atSetpoint),
                    parallel(
                            drive.autoAlign(() -> DriveSetpoints.G.getPose()),
                            elevator.goToSetpoint(ElevatorSetpoints.L3, () -> true))
                        .until(elevator.atSetpoint),
                    parallel(
                            drive.autoAlign(() -> DriveSetpoints.G.getPose()),
                            elevator.goToSetpoint(ElevatorSetpoints.L3, () -> true),
                            ejector.eject(30))
                        .until(ejector.backSensor.negate()),
                    parallel(
                            drive.autoAlign(() -> DriveSetpoints.FAR.getPose()),
                            ejector.eject(40),
                            elevator.goToSetpoint(
                                () -> ElevatorSetpoints.AlgaePopLow, intake.backSensor.negate()))
                        .withTimeout(2)
                        .withName("AlgaePop"),
                    drive.autoAlign(() -> new Pose2d(6.6, 3.866, Rotation2d.kPi)).withTimeout(1))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    // trajectory
    //     .done()
    //     .onTrue(
    //         drive
    //             .autoAlign(() -> trajectory.getFinalPose().get())
    //             .until(drive.atSetpoint)
    //             .andThen(
    //                 elevator
    //                     .goToSetpoint(ElevatorSetpoints.L3, intake.backSensor.negate())
    //                     .until(elevator.atSetpoint)
    //                     .andThen(
    //                         parallel(ejector.eject(15)),
    //                         elevator.goToSetpoint(ElevatorSetpoints.L3,
    // intake.backSensor.negate()))
    //                     .until(ejector.backSensor.negate()))
    //             .andThen(
    //                 elevator
    //                     .goToSetpoint(ElevatorSetpoints.BASE, intake.backSensor.negate())
    //                     .until(elevator.atSetpoint))
    //             .andThen(
    //                 parallel(drive.autoAlign(() -> DriveSetpoints.FAR.getPose()),
    // ejector.eject(30))
    //                     .withTimeout(1))
    //             .withName("autoScore")
    //             .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
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
    var kToHp = routine.trajectory("kToHp");

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
                .until(intake.frontSensor)
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
                        .autoAlign(() -> hpToThird.getFinalPose().get())
                        .until(drive.atSetpoint.and(elevator.atSetpoint)),
                    ejector.eject(20).until(ejector.backSensor.negate()),
                    parallel(
                            drive.autoAlign(DriveSetpoints.CLOSE_LEFT::getPose),
                            elevator
                                .goToSetpoint(
                                    ElevatorSetpoints.AlgaePopLow, intake.backSensor.negate())
                                .asProxy(),
                            ejector.eject(30))
                        .withTimeout(1),
                    kToHp.cmd())
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
    var iToHp = routine.trajectory("iToHp");
    var hpToJ = routine.trajectory("hpToJ");
    var jToHp = routine.trajectory("jToHp");
    var hpToL = routine.trajectory("hpToL");
    var lToHp = routine.trajectory("lToHp");
    var hpToK = routine.trajectory("hpToK");
    var kToHp = routine.trajectory("kToHp");

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
                .until(intake.frontSensor)
                .andThen(hpToJ.cmd())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    hpToJ
        .done()
        .onTrue(
            drive
                .autoAlign(() -> hpToJ.getFinalPose().get())
                .until(drive.atSetpoint)
                .andThen(ejector.eject(20))
                .until(ejector.backSensor.negate())
                .andThen(jToHp.cmd())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    jToHp
        .done()
        .onTrue(
            drive
                .autoAlign(() -> jToHp.getFinalPose().get())
                .until(intake.frontSensor)
                .andThen(hpToL.cmd())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    hpToJ
        .atTime("RaiseElevator")
        .onTrue(
            elevator
                .goToSetpoint(ElevatorSetpoints.L2, intake.backSensor.negate())
                .until(ejector.backSensor.negate()));

    hpToL
        .done()
        .onTrue(
            drive
                .autoAlign(() -> hpToL.getFinalPose().get())
                .until(drive.atSetpoint)
                .andThen(ejector.eject(20))
                .until(ejector.backSensor.negate())
                .andThen(lToHp.cmd())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    lToHp
        .done()
        .onTrue(
            drive
                .autoAlign(() -> lToHp.getFinalPose().get())
                .until(intake.frontSensor)
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
            sequence(
                    drive
                        .autoAlign(() -> hpToK.getFinalPose().get())
                        .until(drive.atSetpoint.and(elevator.atSetpoint)),
                    ejector.eject(20).until(ejector.backSensor.negate()),
                    parallel(
                            drive.autoAlign(DriveSetpoints.CLOSE_LEFT::getPose),
                            elevator
                                .goToSetpoint(
                                    ElevatorSetpoints.AlgaePopLow, intake.backSensor.negate())
                                .asProxy(),
                            ejector.eject(30))
                        .withTimeout(1),
                    kToHp.cmd())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    hpToK
        .atTime("RaiseElevator")
        .onTrue(
            elevator
                .goToSetpoint(ElevatorSetpoints.L3, intake.backSensor.negate())
                .until(ejector.backSensor.negate()));

    return routine;
  }

  public static AutoRoutine fivePieceLeft(
      AutoFactory factory, Drive drive, Ejector ejector, Elevator elevator, Intake intake) {
    var routine = factory.newRoutine("fivePieceLeft");
    var startToI = routine.trajectory("startToI");
    var iToHp = routine.trajectory("iToHp");
    var hpToJ = routine.trajectory("hpToJ");
    var jToHp = routine.trajectory("jToHp");
    var hpToL = routine.trajectory("hpToL");
    var lToHp = routine.trajectory("lToHp");
    var hpToK = routine.trajectory("hpToK");
    var hpToK2 = routine.trajectory("hpToK2");
    var kToHp = routine.trajectory("kToHp");

    routine
        .active()
        .onTrue(
            drive
                .autoAlign(() -> startToI.getInitialPose().get())
                .until(drive.atSetpoint)
                .andThen(startToI.cmd()));

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
                .until(intake.frontSensor)
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
                .andThen(jToHp.cmd())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    jToHp
        .done()
        .onTrue(
            drive
                .autoAlign(() -> jToHp.getFinalPose().get())
                .until(intake.frontSensor)
                .andThen(hpToL.cmd())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    hpToJ
        .atTime("RaiseElevator")
        .onTrue(
            elevator
                .goToSetpoint(ElevatorSetpoints.L2, intake.backSensor.negate())
                .until(ejector.backSensor.negate()));

    hpToL
        .done()
        .onTrue(
            drive
                .autoAlign(() -> hpToL.getFinalPose().get())
                .until(drive.atSetpoint)
                .andThen(ejector.eject(20))
                .until(ejector.backSensor.negate())
                .andThen(lToHp.cmd())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    lToHp
        .done()
        .onTrue(
            drive
                .autoAlign(() -> lToHp.getFinalPose().get())
                .until(intake.frontSensor)
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
            sequence(
                    drive
                        .autoAlign(() -> hpToK.getFinalPose().get())
                        .until(drive.atSetpoint.and(elevator.atSetpoint)),
                    ejector.eject(20).until(ejector.backSensor.negate()),
                    parallel(
                            drive.autoAlign(DriveSetpoints.CLOSE_LEFT::getPose),
                            elevator
                                .goToSetpoint(
                                    ElevatorSetpoints.AlgaePopLow, intake.backSensor.negate())
                                .asProxy(),
                            ejector.eject(30))
                        .withTimeout(0.5),
                    kToHp.cmd())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    hpToK
        .atTime("RaiseElevator")
        .onTrue(
            elevator
                .goToSetpoint(ElevatorSetpoints.L3, intake.backSensor.negate())
                .until(ejector.backSensor.negate()));

    kToHp
        .done()
        .onTrue(
            drive
                .autoAlign(() -> kToHp.getFinalPose().get())
                .until(intake.frontSensor)
                .andThen(hpToK2.cmd())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    hpToK2
        .atTime("RaiseElevator")
        .onTrue(
            elevator
                .goToSetpoint(ElevatorSetpoints.L2, intake.backSensor.negate())
                .until(ejector.backSensor.negate()));

    hpToK2
        .done()
        .onTrue(
            drive
                .autoAlign(() -> hpToK2.getFinalPose().get())
                .until(drive.atSetpoint)
                .andThen(ejector.eject(20))
                .until(ejector.backSensor.negate())
                .andThen(kToHp.cmd())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    return routine;
  }

  public static AutoRoutine fivePieceRight(
      AutoFactory factory, Drive drive, Ejector ejector, Elevator elevator, Intake intake) {
    var routine = factory.newRoutine("fivePieceRight");
    var startToF = routine.trajectory("startToF");
    var fToHp = routine.trajectory("fToHP");
    var hpToE = routine.trajectory("hpToE");
    var eToHp = routine.trajectory("eToHp");
    var hpToD = routine.trajectory("hpToD");
    var dToHp = routine.trajectory("dToHP");
    var hpToC = routine.trajectory("hpToC");
    var hpToC2 = routine.trajectory("hpToC2");
    var cToHp = routine.trajectory("cToHP");

    // routine
    //     .active()
    //     .onTrue(
    //         drive
    //             .autoAlign(() -> startToF.getInitialPose().get())
    //             .until(drive.atSetpoint)
    //             .andThen(startToF.cmd()));
    routine.active().onTrue(startToF.cmd());

    startToF
        .done()
        .onTrue(
            drive
                .autoAlign(() -> startToF.getFinalPose().get())
                .until(drive.atSetpoint.and(elevator.atSetpoint))
                .andThen(ejector.eject(20))
                .until(ejector.backSensor.negate())
                .andThen(fToHp.cmd())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    startToF
        .atTime("RaiseElevator")
        .onTrue(
            elevator
                .goToSetpoint(ElevatorSetpoints.L2, intake.backSensor.negate())
                .until(ejector.backSensor.negate()));

    fToHp
        .done()
        .onTrue(
            drive
                .autoAlign(() -> fToHp.getFinalPose().get())
                .until(intake.frontSensor)
                .andThen(hpToE.cmd())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    hpToE
        .done()
        .onTrue(
            drive
                .autoAlign(() -> hpToE.getFinalPose().get())
                .until(drive.atSetpoint.and(elevator.atSetpoint))
                .andThen(ejector.eject(20))
                .until(ejector.backSensor.negate())
                .andThen(eToHp.cmd())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    eToHp
        .done()
        .onTrue(
            drive
                .autoAlign(() -> eToHp.getFinalPose().get())
                .until(intake.frontSensor)
                .andThen(hpToD.cmd())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    hpToE
        .atTime("RaiseElevator")
        .onTrue(
            elevator
                .goToSetpoint(ElevatorSetpoints.L2, intake.backSensor.negate())
                .until(ejector.backSensor.negate()));

    hpToD
        .done()
        .onTrue(
            drive
                .autoAlign(() -> hpToD.getFinalPose().get())
                .until(drive.atSetpoint)
                .andThen(ejector.eject(20))
                .until(ejector.backSensor.negate())
                .andThen(dToHp.cmd())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    dToHp
        .done()
        .onTrue(
            drive
                .autoAlign(() -> dToHp.getFinalPose().get())
                .until(intake.frontSensor)
                .andThen(hpToC.cmd())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    hpToD
        .atTime("RaiseElevator")
        .onTrue(
            elevator
                .goToSetpoint(ElevatorSetpoints.L3, intake.backSensor.negate())
                .until(ejector.backSensor.negate()));

    hpToC
        .done()
        .onTrue(
            sequence(
                    drive
                        .autoAlign(() -> hpToC.getFinalPose().get())
                        .until(drive.atSetpoint.and(elevator.atSetpoint)),
                    ejector.eject(20).until(ejector.backSensor.negate()),
                    parallel(
                            drive.autoAlign(DriveSetpoints.CLOSE_RIGHT::getPose),
                            elevator
                                .goToSetpoint(
                                    ElevatorSetpoints.AlgaePopLow, intake.backSensor.negate())
                                .asProxy(),
                            ejector.eject(30))
                        .withTimeout(0.5),
                    cToHp.cmd())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    hpToC
        .atTime("RaiseElevator")
        .onTrue(
            elevator
                .goToSetpoint(ElevatorSetpoints.L3, intake.backSensor.negate())
                .until(ejector.backSensor.negate()));

    cToHp
        .done()
        .onTrue(
            drive
                .autoAlign(() -> cToHp.getFinalPose().get())
                .until(intake.frontSensor)
                .andThen(hpToC2.cmd())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    hpToC2
        .atTime("RaiseElevator")
        .onTrue(
            elevator
                .goToSetpoint(ElevatorSetpoints.L2, intake.backSensor.negate())
                .until(ejector.backSensor.negate()));

    hpToC2
        .done()
        .onTrue(
            drive
                .autoAlign(() -> hpToC2.getFinalPose().get())
                .until(drive.atSetpoint)
                .andThen(ejector.eject(20))
                .until(ejector.backSensor.negate())
                .andThen(cToHp.cmd())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    return routine;
  }
}
