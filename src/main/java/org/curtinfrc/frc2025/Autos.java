// package org.curtinfrc.frc2025;
//
// import edu.wpi.first.math.Pair;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import java.util.EnumMap;
// import java.util.Map;
// import java.util.Optional;
// import java.util.Set;
// import java.util.stream.Collectors;
// import org.curtinfrc.frc2025.Autos.AlgaePoppedStates.HasAlgae;
// import org.curtinfrc.frc2025.Constants.Setpoint;
// import org.curtinfrc.frc2025.subsystems.drive.Drive;
// import org.curtinfrc.frc2025.subsystems.drive.DriveConstants.DriveSetpoints;
// import org.curtinfrc.frc2025.subsystems.ejector.Ejector;
// import org.curtinfrc.frc2025.subsystems.elevator.Elevator;
// import org.curtinfrc.frc2025.subsystems.elevator.ElevatorConstants.ElevatorSetpoints;
// import org.curtinfrc.frc2025.subsystems.intake.Intake;
// import org.curtinfrc.frc2025.subsystems.popper.Popper;
// import org.littletonrobotics.junction.Logger;
//
// public class Autos {
//   public static class AlgaePoppedStates {
//     enum AlgaeLocations {
//       AB,
//       CD,
//       EF,
//       GH,
//       IJ,
//       KL;
//
//       public static AlgaeLocations from(DriveSetpoints point) {
//         return switch (point) {
//           case A, B -> AB;
//           case C, D -> CD;
//           case E, F -> EF;
//           case G, H -> GH;
//           case I, J -> IJ;
//           case K, L -> KL;
//           default -> throw new IllegalArgumentException("Unknown DriveSetpoint: " + point);
//         };
//       }
//     }
//
//     enum HasAlgae {
//       HAS,
//       NOT,
//       NONE
//     }
//
//     private final Map<AlgaeLocations, Pair<HasAlgae, HasAlgae>> locations =
//         new EnumMap<>(AlgaeLocations.class);
//
//     public AlgaePoppedStates() {
//       for (AlgaeLocations loc : AlgaeLocations.values()) {
//         if (isHigh(loc)) {
//           locations.put(loc, Pair.of(HasAlgae.NONE, HasAlgae.HAS));
//         } else {
//           locations.put(loc, Pair.of(HasAlgae.HAS, HasAlgae.NONE));
//         }
//       }
//
//       Logger.recordOutput("Locations", getFormattedMap(locations));
//     }
//
//     private String getFormattedMap(
//         Map<AlgaePoppedStates.AlgaeLocations, Pair<HasAlgae, HasAlgae>> map) {
//       return map.entrySet().stream()
//           .map(
//               entry ->
//                   entry.getKey()
//                       + ": [Low: "
//                       + entry.getValue().getFirst()
//                       + ", High: "
//                       + entry.getValue().getSecond()
//                       + "]")
//           .collect(Collectors.joining(", ", "{", "}"));
//     }
//
//     public void pop(AlgaeLocations loc, boolean high) {
//       locations.put(
//           loc, Pair.of(high ? HasAlgae.NONE : HasAlgae.NOT, high ? HasAlgae.NOT :
// HasAlgae.NONE));
//     }
//
//     public Pair<HasAlgae, HasAlgae> isPopped(AlgaeLocations loc) {
//       return locations.getOrDefault(loc, Pair.of(HasAlgae.NONE, HasAlgae.NONE));
//     }
//
//     public static boolean isHigh(AlgaeLocations loc) {
//       switch (loc) {
//         case AB, EF, IJ:
//           return true;
//         default:
//           return false;
//       }
//     }
//   }
//
//   private static final AlgaePoppedStates state = new AlgaePoppedStates();
//
//   private static class RepulsorAuto {
//     private final Setpoint[] setpoints;
//     private final DriveSetpoints hpSetpoint;
//     private final Drive drive;
//     private final Elevator elevator;
//     private final Popper popper;
//     private final Ejector ejector;
//     private final Intake intake;
//
//     public RepulsorAuto(
//         Drive drive,
//         Elevator elevator,
//         Popper popper,
//         Ejector ejector,
//         Intake intake,
//         DriveSetpoints hpSetpoint,
//         Setpoint... reefSetpoints) {
//       this.setpoints = reefSetpoints;
//       this.drive = drive;
//       this.hpSetpoint = hpSetpoint;
//       this.elevator = elevator;
//       this.popper = popper;
//       this.ejector = ejector;
//       this.intake = intake;
//     }
//
//     public Command build() {
//       return Commands.defer(
//           () -> {
//             int[] stepCounter = {1};
//             Logger.recordOutput("Auto/Sequence " + stepCounter[0], "Starting Auto Sequence");
//             stepCounter[0]++;
//
//             Command sequence = Commands.sequence().withName("Initial Empty Command");
//
//             int i = 0;
//             for (Setpoint setpoint : setpoints) {
//               i++;
//               DriveSetpoints driveSetpoint = setpoint.driveSetpoint();
//               AlgaePoppedStates.AlgaeLocations loc =
//                   AlgaePoppedStates.AlgaeLocations.from(driveSetpoint);
//
//               boolean isHigh = AlgaePoppedStates.isHigh(loc);
//               Logger.recordOutput("isHigh", isHigh);
//               if (isHigh
//                   ? state.isPopped(loc).getSecond() == HasAlgae.HAS
//                       && setpoint.elevatorSetpoint() == ElevatorSetpoints.L3
//                   : state.isPopped(loc).getFirst() == HasAlgae.HAS
//                       && setpoint.elevatorSetpoint() == ElevatorSetpoints.L2) {
//                 Logger.recordOutput("Auto/Sequence " + stepCounter[0], "Align to " +
// driveSetpoint);
//                 Command alignCommand =
//                     drive
//                         .autoAlign(
//                             driveSetpoint, Optional.empty(), Optional.empty(), Optional.empty())
//                         .until(drive.atSetpoint)
//                         .withName("Align to " + driveSetpoint);
//                 stepCounter[0]++;
//
//                 sequence = sequence.andThen(alignCommand);
//
//                 Logger.recordOutput("Auto/Sequence " + stepCounter[0], "Pop Algae at " + loc);
//                 Command popCommand =
//                     elevator
//                         .goToSetpoint(ElevatorSetpoints.getPopPoint(setpoint.elevatorSetpoint()))
//                         .until(elevator.atSetpoint)
//                         .withName("Pop Algae at " + loc);
//                 stepCounter[0]++;
//
//                 sequence = sequence.andThen(popCommand);
//                 state.pop(loc, isHigh);
//               }
//
//               Logger.recordOutput("Auto/Sequence " + stepCounter[0], "Align to " +
// driveSetpoint);
//               Command alignCommand =
//                   drive
//                       .autoAlign(
//                           driveSetpoint, Optional.empty(), Optional.empty(), Optional.empty())
//                       .until(drive.atSetpoint)
//                       .withName("Align to " + driveSetpoint);
//               stepCounter[0]++;
//
//               sequence = sequence.andThen(alignCommand);
//
//               sequence =
//                   sequence.andThen(
//                       elevator
//                           .goToSetpoint(setpoint.elevatorSetpoint())
//                           .until(ejector.backSensor.negate())
//                           .withName("Go Up " + loc));
//
//               Logger.recordOutput(
//                   "Auto/Sequence " + stepCounter[0], "Return to HP at " + hpSetpoint);
//               Command returnToHPCommand =
//                   drive
//                       .autoAlign(hpSetpoint, Optional.empty(), Optional.empty(),
// Optional.empty())
//                       .until(drive.atSetpoint.and(intake.frontSensor))
//                       .withName("Return to HP at " + hpSetpoint);
//               stepCounter[0]++;
//
//               if (setpoints.length != i) {
//                 sequence = sequence.andThen(returnToHPCommand);
//               }
//             }
//
//             Logger.recordOutput("Auto/Sequence " + stepCounter[0], "Finished Auto Sequence");
//             return sequence;
//           },
//           Set.of(drive, elevator, popper));
//     }
//   }
//
//   private final Drive drive;
//   private final Elevator elevator;
//   private final Popper popper;
//   private final Ejector ejector;
//   private final Intake intake;
//
//   public Autos(Drive drive, Elevator elevator, Popper popper, Ejector ejector, Intake intake) {
//     this.drive = drive;
//     this.elevator = elevator;
//     this.popper = popper;
//     this.ejector = ejector;
//     this.intake = intake;
//   }
//
//   public Command basicAuto() {
//     return new RepulsorAuto(
//             drive,
//             elevator,
//             popper,
//             ejector,
//             intake,
//             DriveSetpoints.LEFT_HP,
//             new Setpoint(ElevatorSetpoints.L2, DriveSetpoints.A),
//             new Setpoint(ElevatorSetpoints.L2, DriveSetpoints.B))
//         .build();
//   }
// }
