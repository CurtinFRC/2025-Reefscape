package org.curtinfrc.frc2025;

import static org.curtinfrc.frc2025.subsystems.drive.DriveConstants.DriveSetpoints.*;
import static org.curtinfrc.frc2025.subsystems.vision.VisionConstants.*;

import choreo.auto.AutoFactory;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.curtinfrc.frc2025.Constants.Mode;
import org.curtinfrc.frc2025.Constants.Setpoint;
import org.curtinfrc.frc2025.generated.CompTunerConstants;
import org.curtinfrc.frc2025.generated.DevTunerConstants;
import org.curtinfrc.frc2025.subsystems.climber.Climber;
import org.curtinfrc.frc2025.subsystems.climber.ClimberConstants;
import org.curtinfrc.frc2025.subsystems.climber.ClimberIO;
import org.curtinfrc.frc2025.subsystems.climber.ClimberIOComp;
import org.curtinfrc.frc2025.subsystems.climber.ClimberIOSim;
import org.curtinfrc.frc2025.subsystems.drive.Drive;
import org.curtinfrc.frc2025.subsystems.drive.DriveConstants.DriveSetpoints;
import org.curtinfrc.frc2025.subsystems.drive.GyroIO;
import org.curtinfrc.frc2025.subsystems.drive.GyroIOPigeon2;
import org.curtinfrc.frc2025.subsystems.drive.ModuleIO;
import org.curtinfrc.frc2025.subsystems.drive.ModuleIOSim;
import org.curtinfrc.frc2025.subsystems.drive.ModuleIOTalonFX;
import org.curtinfrc.frc2025.subsystems.ejector.Ejector;
import org.curtinfrc.frc2025.subsystems.ejector.EjectorIO;
import org.curtinfrc.frc2025.subsystems.ejector.EjectorIOComp;
import org.curtinfrc.frc2025.subsystems.ejector.EjectorIOSim;
import org.curtinfrc.frc2025.subsystems.elevator.Elevator;
import org.curtinfrc.frc2025.subsystems.elevator.ElevatorConstants.ElevatorSetpoints;
import org.curtinfrc.frc2025.subsystems.elevator.ElevatorIO;
import org.curtinfrc.frc2025.subsystems.elevator.ElevatorIOComp;
import org.curtinfrc.frc2025.subsystems.elevator.ElevatorIOSim;
import org.curtinfrc.frc2025.subsystems.intake.Intake;
import org.curtinfrc.frc2025.subsystems.intake.IntakeIO;
import org.curtinfrc.frc2025.subsystems.intake.IntakeIOComp;
import org.curtinfrc.frc2025.subsystems.intake.IntakeIOSim;
import org.curtinfrc.frc2025.subsystems.leds.LEDs;
import org.curtinfrc.frc2025.subsystems.leds.LEDsIO;
import org.curtinfrc.frc2025.subsystems.leds.LEDsIOComp;
import org.curtinfrc.frc2025.subsystems.vision.Vision;
import org.curtinfrc.frc2025.subsystems.vision.VisionIO;
import org.curtinfrc.frc2025.subsystems.vision.VisionIOLimelight;
import org.curtinfrc.frc2025.subsystems.vision.VisionIOPhotonVision;
import org.curtinfrc.frc2025.subsystems.vision.VisionIOPhotonVisionSim;
import org.curtinfrc.frc2025.util.AutoChooser;
import org.curtinfrc.frc2025.util.ButtonBoard;
import org.curtinfrc.frc2025.util.PhoenixUtil;
import org.curtinfrc.frc2025.util.VirtualSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  // Subsystems
  private Drive drive;
  private Vision vision;
  private Intake intake;
  private LEDs leds;
  private Elevator elevator;
  private Ejector ejector;
  private Climber climber;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final Alert controllerDisconnected =
      new Alert("Driver controller disconnected!", AlertType.kError);
  private final ButtonBoard board = new ButtonBoard(1);

  // Auto stuff
  private final AutoChooser autoChooser;
  private final AutoFactory factory;
  // private final Autos autos;

  private List<Pose2d> leftSetpoints;
  private List<Pose2d> rightSetpoints;
  private List<Pose2d> l1Setpoints;
  private List<Pose2d> algaeSetpoints;

  @AutoLogOutput(key = "Robot/Overridden")
  private boolean overridden = false;

  @AutoLogOutput(key = "Robot/Overide")
  private final Trigger override = new Trigger(() -> overridden);

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("RobotType", Constants.robotType.toString());
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0 -> Logger.recordMetadata("GitDirty", "All changes committed");
      case 1 -> Logger.recordMetadata("GitDirty", "Uncommitted changes");
      default -> Logger.recordMetadata("GitDirty", "Unknown");
    }

    switch (Constants.getMode()) {
      case REAL -> {
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
      }

      case SIM -> {
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
      }

      case REPLAY -> {
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
      }
    }

    SignalLogger.start();
    SignalLogger.setPath("/U/logs");
    DataLogManager.start();
    Logger.registerURCL(URCL.startExternal());
    // Start AdvantageKit logger
    Logger.start();

    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.robotType) {
        case COMPBOT -> {
          // Real robot, instantiate hardware IO implementationsRobot
          drive =
              new Drive(
                  new GyroIOPigeon2(CompTunerConstants.DrivetrainConstants),
                  new ModuleIOTalonFX(CompTunerConstants.FrontLeft),
                  new ModuleIOTalonFX(CompTunerConstants.FrontRight),
                  new ModuleIOTalonFX(CompTunerConstants.BackLeft),
                  new ModuleIOTalonFX(CompTunerConstants.BackRight));
          vision =
              new Vision(
                  drive::addVisionMeasurement,
                  drive::getRotation,
                  new VisionIOPhotonVision(camera0Name, robotToCamera0),
                  new VisionIOPhotonVision(camera1Name, robotToCamera1),
                  // new VisionIOPhotonVision(camera2Name, robotToCamera2),
                  new VisionIO() {},
                  new VisionIOPhotonVision(camera3Name, robotToCamera3));
          elevator = new Elevator(new ElevatorIOComp());
          intake = new Intake(new IntakeIOComp());
          ejector = new Ejector(new EjectorIOComp());
          climber = new Climber(new ClimberIOComp());
          leds = new LEDs(new LEDsIOComp());
        }

        case DEVBOT -> {
          // Real robot, instantiate hardware IO implementationsRobot
          drive =
              new Drive(
                  new GyroIOPigeon2(DevTunerConstants.DrivetrainConstants),
                  new ModuleIOTalonFX(DevTunerConstants.FrontLeft),
                  new ModuleIOTalonFX(DevTunerConstants.FrontRight),
                  new ModuleIOTalonFX(DevTunerConstants.BackLeft),
                  new ModuleIOTalonFX(DevTunerConstants.BackRight));
          vision =
              new Vision(
                  drive::addVisionMeasurement,
                  drive::getRotation,
                  new VisionIO() {},
                  new VisionIOLimelight(camera1Name, drive::getRotation),
                  new VisionIOLimelight(camera2Name, drive::getRotation),
                  new VisionIO() {});
          elevator = new Elevator(new ElevatorIO() {});
          intake = new Intake(new IntakeIO() {});
          ejector = new Ejector(new EjectorIO() {});
          climber = new Climber(new ClimberIO() {});
          leds = new LEDs(new LEDsIO() {});
        }

        case SIMBOT -> {
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(CompTunerConstants.FrontLeft),
                  new ModuleIOSim(CompTunerConstants.FrontRight),
                  new ModuleIOSim(CompTunerConstants.BackLeft),
                  new ModuleIOSim(CompTunerConstants.BackRight));
          vision =
              new Vision(
                  drive::addVisionMeasurement,
                  drive::getRotation,
                  new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                  new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose),
                  new VisionIOPhotonVisionSim(camera2Name, robotToCamera2, drive::getPose),
                  new VisionIOPhotonVisionSim(camera3Name, robotToCamera3, drive::getPose));

          elevator = new Elevator(new ElevatorIOSim());
          intake = new Intake(new IntakeIOSim());
          ejector = new Ejector(new EjectorIOSim());
          climber = new Climber(new ClimberIOSim());
          leds = new LEDs(new LEDsIO() {});
        }
      }
    } else {
      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});

      vision =
          new Vision(
              drive::addVisionMeasurement,
              drive::getRotation,
              new VisionIO() {},
              new VisionIO() {},
              new VisionIO() {},
              new VisionIO() {});

      elevator = new Elevator(new ElevatorIO() {});
      intake = new Intake(new IntakeIO() {});
      ejector = new Ejector(new EjectorIO() {});
      climber = new Climber(new ClimberIO() {});
      leds = new LEDs(new LEDsIO() {});
    }

    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

    autoChooser = new AutoChooser("Auto Chooser");
    factory =
        new AutoFactory(
            drive::getPose,
            drive::setPose,
            drive::followTrajectory,
            true,
            drive,
            drive::logTrajectory);

    autoChooser.addRoutine("Test Path", () -> Autos.path("Test Path", factory, drive));
    autoChooser.addRoutine(
        "One Piece Centre", () -> Autos.onePieceCentre(factory, drive, ejector, elevator, intake));
    autoChooser.addRoutine(
        "One Piece Left", () -> Autos.onePieceLeft(factory, drive, ejector, elevator, intake));
    autoChooser.addRoutine(
        "Two Piece Left", () -> Autos.twoPieceLeft(factory, drive, ejector, elevator, intake));
    autoChooser.addRoutine(
        "Three Piece Left", () -> Autos.threePieceLeft(factory, drive, ejector, elevator, intake));
    autoChooser.addRoutine(
        "Four Piece Left", () -> Autos.fourPieceLeft(factory, drive, ejector, elevator, intake));
    autoChooser.addRoutine(
        "Five Piece Left", () -> Autos.fivePieceLeft(factory, drive, ejector, elevator, intake));
    autoChooser.addRoutine(
        "Five Piece Right", () -> Autos.fivePieceRight(factory, drive, ejector, elevator, intake));

    autoChooser.addCmd("Test Auto", this::testAuto);

    // Set up SysId routines
    autoChooser.addCmd(
        "Drive Wheel Radius Characterization", () -> drive.wheelRadiusCharacterization());
    autoChooser.addCmd(
        "Drive Simple FF Characterization", () -> drive.feedforwardCharacterization());

    autoChooser.addCmd(
        "Drive Translation SysId (Quasistatic Forward)",
        () -> drive.sysIdTranslationQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addCmd(
        "Drive Translation SysId (Quasistatic Reverse)",
        () -> drive.sysIdTranslationQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addCmd(
        "Drive Translation SysId (Dynamic Forward)",
        () -> drive.sysIdTranslationDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addCmd(
        "Drive Translation SysId (Dynamic Reverse)",
        () -> drive.sysIdTranslationDynamic(SysIdRoutine.Direction.kReverse));

    autoChooser.addCmd(
        "Drive Steer SysId (Quasistatic Forward)",
        () -> drive.sysIdSteerQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addCmd(
        "Drive Steer SysId (Quasistatic Reverse)",
        () -> drive.sysIdSteerQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addCmd(
        "Drive Steer SysId (Dynamic Forward)",
        () -> drive.sysIdSteerDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addCmd(
        "Drive Steer SysId (Dynamic Reverse)",
        () -> drive.sysIdSteerDynamic(SysIdRoutine.Direction.kReverse));

    RobotModeTriggers.autonomous()
        .whileTrue(autoChooser.selectedCommandScheduler().withName("AutoCMD"));

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        drive.joystickDrive(
            () -> controller.getLeftY(),
            () -> controller.getLeftX(),
            () -> controller.getRightX()));

    drive
        .atSetpoint
        .and(elevator.atSetpoint)
        .and(elevator.isNotAtCollect)
        .whileTrue(ejector.eject(15).until(ejector.backSensor.negate()));

    drive.atSetpoint.whileTrue(leds.setBlue());

    controller
        .rightBumper()
        .or(controller.leftBumper())
        .and(override.negate())
        .whileTrue(
            elevator
                .goToSetpoint(ElevatorSetpoints.L2, intake.backSensor.negate())
                .until(ejector.backSensor.negate()));
    controller
        .rightTrigger()
        .or(controller.leftTrigger())
        .and(override.negate())
        .whileTrue(
            elevator
                .goToSetpoint(ElevatorSetpoints.L3, intake.backSensor.negate())
                .until(ejector.backSensor.negate()));

    controller
        .rightBumper()
        .or(controller.rightTrigger())
        .and(override.negate())
        .whileTrue(
            drive
                .autoAlignWithOverride(
                    () -> DriveSetpoints.closest(drive::getPose, rightSetpoints),
                    () -> controller.getLeftY(),
                    () -> controller.getLeftX(),
                    () -> controller.getRightX())
                .until(ejector.backSensor.negate()));

    controller.leftBumper().or(controller.leftTrigger()).and(override).whileTrue(ejector.eject(30));
    controller
        .leftBumper()
        .and(override)
        .whileTrue(
            elevator.goToSetpoint(ElevatorSetpoints.AlgaePopLow, intake.backSensor.negate()));
    controller
        .leftTrigger()
        .and(override)
        .whileTrue(
            elevator.goToSetpoint(ElevatorSetpoints.AlgaePopHigh, intake.backSensor.negate()));
    controller
        .rightBumper()
        .and(override)
        .whileTrue(elevator.goToSetpoint(ElevatorSetpoints.L2, intake.backSensor.negate()));
    controller
        .rightTrigger()
        .and(override)
        .whileTrue(elevator.goToSetpoint(ElevatorSetpoints.L3, intake.backSensor.negate()));

    controller
        .leftBumper()
        .or(controller.leftTrigger())
        .and(override.negate())
        .whileTrue(
            drive
                .autoAlignWithOverride(
                    () -> DriveSetpoints.closest(drive::getPose, leftSetpoints),
                    () -> controller.getLeftY(),
                    () -> controller.getLeftX(),
                    () -> controller.getRightX())
                .until(ejector.backSensor.negate()));

    climber.stalled.onTrue(
        climber
            .stop()
            .andThen(
                elevator
                    .goToClimberSetpoint(ElevatorSetpoints.climbed, intake.backSensor.negate())
                    .withTimeout(0.5)
                    .andThen(
                        Commands.parallel(
                            climber.engage(),
                            elevator.goToClimberSetpoint(
                                ElevatorSetpoints.climbed, intake.backSensor.negate())))
                    .until(elevator.atClimbSetpoint)
                    .andThen(Commands.parallel(climber.engage(), elevator.stop().repeatedly()))));

    intake.setDefaultCommand(intake.intake());
    ejector.setDefaultCommand(
        ejector.stop().withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    elevator.setDefaultCommand(
        elevator
            .goToSetpoint(ElevatorSetpoints.BASE, intake.backSensor.negate())
            .withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    climber.setDefaultCommand(climber.stop());

    ejector.backSensor.onFalse(
        Commands.run(() -> controller.setRumble(RumbleType.kBothRumble, 0.5))
            .withTimeout(0.5)
            .andThen(Commands.runOnce(() -> controller.setRumble(RumbleType.kBothRumble, 0.0))));
    intake.frontSensor.onTrue(
        Commands.run(() -> controller.setRumble(RumbleType.kBothRumble, 0.5))
            .withTimeout(0.5)
            .andThen(Commands.runOnce(() -> controller.setRumble(RumbleType.kBothRumble, 0.0))));

    controller
        .povLeft()
        .and(
            controller
                .rightBumper()
                .or(controller.leftBumper())
                .or(controller.rightTrigger())
                .or(controller.leftTrigger())
                .negate())
        .and(override.negate())
        .whileTrue(
            Commands.parallel(
                    drive.autoAlignWithOverride(
                        () -> DriveSetpoints.closest(drive::getPose, algaeSetpoints),
                        () -> controller.getLeftY(),
                        () -> controller.getLeftX(),
                        () -> controller.getRightX()),
                    ejector.eject(40),
                    elevator.goToSetpoint(
                        () -> {
                          return switch (DriveSetpoints.closest(drive::getPose, leftSetpoints)) {
                            case A, B -> ElevatorSetpoints.AlgaePopHigh;
                            case C, D -> ElevatorSetpoints.AlgaePopLow;
                            case E, F -> ElevatorSetpoints.AlgaePopHigh;
                            case G, H -> ElevatorSetpoints.AlgaePopLow;
                            case I, J -> ElevatorSetpoints.AlgaePopHigh;
                            case K, L -> ElevatorSetpoints.AlgaePopLow;
                            default -> ElevatorSetpoints.AlgaePopLow;
                          };
                        },
                        intake.backSensor.negate()))
                .withName("AlgaePop")
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    controller
        .povRight()
        .and(override.negate())
        .whileTrue(
            Commands.sequence(
                Commands.parallel(
                        drive.autoAlignWithOverride(
                            () -> DriveSetpoints.closest(drive::getPose, l1Setpoints),
                            () -> controller.getLeftY(),
                            () -> controller.getLeftX(),
                            () -> controller.getRightX()),
                        elevator.goToSetpoint(ElevatorSetpoints.L1, intake.backSensor.negate()))
                    .until(elevator.atSetpoint.and(drive.atSetpoint)),
                Commands.parallel(
                    drive.autoAlignWithOverride(
                        () -> DriveSetpoints.closest(drive::getPose, l1Setpoints),
                        () -> controller.getLeftY(),
                        () -> controller.getLeftX(),
                        () -> controller.getRightX()),
                    elevator.goToSetpoint(ElevatorSetpoints.L1, intake.backSensor.negate()),
                    ejector.eject(25))));
    controller
        .povLeft()
        .and(
            controller
                .rightBumper()
                .or(controller.leftBumper())
                .or(controller.rightTrigger())
                .or(controller.leftTrigger()))
        .whileTrue(ejector.eject(15).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    intake
        .backSensor
        .and(elevator.isNotAtCollect.negate())
        .and(elevator.atSetpoint)
        .whileTrue(ejector.eject(15));

    intake
        .backSensor
        .negate()
        .and(intake.frontSensor.negate())
        .and(ejector.frontSensor.negate())
        .and(ejector.backSensor.negate())
        .whileTrue(leds.setPink())
        .whileFalse(leds.setGreen());

    intake.backSensor.negate().and(ejector.frontSensor).whileTrue(ejector.stop());

    intake
        .backSensor
        .negate()
        .and(ejector.frontSensor.negate())
        .and(ejector.backSensor)
        .and(elevator.isNotAtCollect.negate())
        .whileTrue(
            ejector
                .eject(-2)
                .until(ejector.frontSensor)
                .andThen(ejector.stop())
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    intake.motorStalled.whileTrue(intake.intake(-4));
    ejector.backSensor.whileTrue(intake.stop());

    // Reset gyro to 0° when B button is pressed
    controller
        .y()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
    controller
        .x()
        .onTrue(
            Commands.sequence(
                climber.disengage(),
                climber.goToSetpoint(ClimberConstants.targetPositionRotationsIn),
                elevator.goToSetpoint(ElevatorSetpoints.climbPrep, intake.backSensor.negate())));

    controller // climb attempt
        .a()
        .and(() -> climber.climberDeployed)
        .onTrue(
            elevator
                .goToSetpoint(ElevatorSetpoints.climbAttempt, intake.backSensor.negate())
                .until(elevator.atSetpoint)
                .andThen(climber.goToSetpoint(ClimberConstants.targetPositionRotationsOut))
                .andThen(climber.goToSetpoint(ClimberConstants.targetPositionRotationsIn))
                .andThen(
                    new ScheduleCommand(
                        elevator.goToSetpoint(
                            ElevatorSetpoints.climbPrep, intake.backSensor.negate()))));

    controller.b().onTrue(Commands.runOnce(() -> overridden = !overridden));

    new Trigger(this::isEnabled).onTrue(climber.disengage());

    CommandScheduler.getInstance().onCommandInitialize(this::commandStarted);
    CommandScheduler.getInstance().onCommandFinish(this::commandEnded);
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            (interrupted, interrupting) -> {
              interrupting.ifPresent(
                  interrupter -> runningInterrupters.put(interrupter, interrupted));
              commandEnded(interrupted);
            });
  }

  private final Set<Command> runningNonInterrupters = new HashSet<>();
  private final Map<Command, Command> runningInterrupters = new HashMap<>();
  private final Map<Subsystem, Command> requiredSubsystems = new HashMap<>();

  private void commandStarted(final Command command) {
    if (!runningInterrupters.containsKey(command)) {
      runningNonInterrupters.add(command);
    }

    for (final Subsystem subsystem : command.getRequirements()) {
      requiredSubsystems.put(subsystem, command);
    }
  }

  private void commandEnded(final Command command) {
    runningNonInterrupters.remove(command);
    runningInterrupters.remove(command);

    for (final Subsystem subsystem : command.getRequirements()) {
      requiredSubsystems.remove(subsystem);
    }
  }

  private final StringBuilder subsystemsBuilder = new StringBuilder();

  private String getCommandName(Command command) {
    subsystemsBuilder.setLength(0);
    int j = 1;
    for (final Subsystem subsystem : command.getRequirements()) {
      subsystemsBuilder.append(subsystem.getName());
      if (j < command.getRequirements().size()) {
        subsystemsBuilder.append(",");
      }

      j++;
    }
    var finalName = command.getName();
    if (j > 1) {
      finalName += " (" + subsystemsBuilder + ")";
    }
    return finalName;
  }

  private void logRunningCommands() {
    Logger.recordOutput("CommandScheduler/Running/.type", "Alerts");

    //    final String[] runningCommands = new String[runningNonInterrupters.size()];
    //    int i = 0;
    //    for (final Command command : runningNonInterrupters) {
    //      runningCommands[i] = getCommandName(command);
    //      i++;
    //    }
    final ArrayList<String> runningCommands = new ArrayList<>();
    final ArrayList<String> runningDefaultCommands = new ArrayList<>();
    for (final Command command : runningNonInterrupters) {
      boolean isDefaultCommand = false;
      for (Subsystem subsystem : command.getRequirements()) {
        if (subsystem.getDefaultCommand() == command) {
          runningDefaultCommands.add(getCommandName(command));
          isDefaultCommand = true;
          break;
        }
      }
      if (!isDefaultCommand) {
        runningCommands.add(getCommandName(command));
      }
    }
    Logger.recordOutput(
        "CommandScheduler/Running/warnings", runningCommands.toArray(new String[0]));
    Logger.recordOutput(
        "CommandScheduler/Running/infos", runningDefaultCommands.toArray(new String[0]));

    final String[] interrupters = new String[runningInterrupters.size()];
    int j = 0;
    for (final Map.Entry<Command, Command> entry : runningInterrupters.entrySet()) {
      final Command interrupter = entry.getKey();
      final Command interrupted = entry.getValue();

      interrupters[j] = getCommandName(interrupter) + " interrupted " + getCommandName(interrupted);
      j++;
    }

    Logger.recordOutput("CommandScheduler/Running/errors", interrupters);
  }

  private void logRequiredSubsystems() {
    Logger.recordOutput("CommandScheduler/Subsystems/.type", "Alerts");

    final String[] subsystems = new String[requiredSubsystems.size()];
    {
      int i = 0;
      for (final Map.Entry<Subsystem, Command> entry : requiredSubsystems.entrySet()) {
        final Subsystem required = entry.getKey();
        final Command command = entry.getValue();

        subsystems[i] = required.getName() + " (" + command.getName() + ")";
        i++;
      }
    }
    Logger.recordOutput("CommandScheduler/Subsystems/infos", subsystems);
  }

  @Override
  public void driverStationConnected() {
    leftSetpoints =
        List.of(A.getPose(), C.getPose(), E.getPose(), G.getPose(), I.getPose(), K.getPose());

    rightSetpoints =
        List.of(B.getPose(), D.getPose(), F.getPose(), H.getPose(), J.getPose(), L.getPose());

    l1Setpoints =
        List.of(
            A_LEFTL1.getPose(),
            A_RIGHTL1.getPose(),
            C_LEFTL1.getPose(),
            C_RIGHTL1.getPose(),
            E_LEFTL1.getPose(),
            E_RIGHTL1.getPose(),
            G_LEFTL1.getPose(),
            G_RIGHTL1.getPose(),
            I_LEFTL1.getPose(),
            I_RIGHTL1.getPose(),
            K_LEFTL1.getPose(),
            K_RIGHTL1.getPose());

    algaeSetpoints =
        List.of(
            CLOSE.getPose(),
            FAR.getPose(),
            CLOSE_LEFT.getPose(),
            FAR_LEFT.getPose(),
            CLOSE_RIGHT.getPose(),
            FAR_RIGHT.getPose());
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Switch thread to high priority to improve loop timing
    Threads.setCurrentThreadPriority(true, 99);

    PhoenixUtil.refreshAll();

    controllerDisconnected.set(!controller.isConnected());

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();
    logRunningCommands();
    logRequiredSubsystems();
    Logger.recordOutput(
        "LoggedRobot/MemoryUsageMb",
        (Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory()) / 1e6);

    // Runs virtual subsystems
    VirtualSubsystem.periodicAll();

    // Return to normal thread priority
    Threads.setCurrentThreadPriority(false, 10);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    autoChooser.periodic();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  public Command onePiece() {
    return drive
        .autoAlign(() -> DriveSetpoints.C.getPose())
        .until(drive.atSetpoint)
        .andThen(
            elevator
                .goToSetpoint(ElevatorSetpoints.L2, intake.backSensor.negate())
                .until(elevator.atSetpoint)
                .andThen(
                    Commands.parallel(
                        elevator.goToSetpoint(ElevatorSetpoints.L2, intake.backSensor.negate()),
                        ejector.eject(8))))
        .until(ejector.backSensor.negate());
  }

  public Command testAuto() {
    return node(new Setpoint(ElevatorSetpoints.L2, DriveSetpoints.K));
  }

  public Command threeCoralRight() {
    // E F B
    return node(new Setpoint(ElevatorSetpoints.L2, DriveSetpoints.E))
        .andThen(intake(DriveSetpoints.RIGHT_HP))
        .andThen(node(new Setpoint(ElevatorSetpoints.L2, DriveSetpoints.F)))
        .andThen(intake(DriveSetpoints.RIGHT_HP))
        .andThen(node(new Setpoint(ElevatorSetpoints.L2, DriveSetpoints.B)))
        .andThen(intake(DriveSetpoints.RIGHT_HP));
  }

  public Command threeCoralLeft() {
    // I J A
    return node(new Setpoint(ElevatorSetpoints.L2, DriveSetpoints.I))
        .andThen(intake(DriveSetpoints.LEFT_HP))
        .andThen(node(new Setpoint(ElevatorSetpoints.L2, DriveSetpoints.J)))
        .andThen(intake(DriveSetpoints.LEFT_HP))
        .andThen(node(new Setpoint(ElevatorSetpoints.L2, DriveSetpoints.A)))
        .andThen(intake(DriveSetpoints.LEFT_HP));
  }

  private Command node(Setpoint point) {
    return drive
        .autoAlign(() -> point.driveSetpoint().getPose())
        .until(drive.atSetpoint)
        .andThen(
            Commands.parallel(
                drive.autoAlign(() -> point.driveSetpoint().getPose()),
                elevator.goToSetpoint(point.elevatorSetpoint(), intake.backSensor.negate())))
        .withName("firststep")
        .until(elevator.atSetpoint)
        .withName("GetToAutoPosition")
        .andThen(
            Commands.parallel(
                drive.autoAlign(() -> point.driveSetpoint().getPose()),
                ejector.eject(15).asProxy(),
                elevator.goToSetpoint(point.elevatorSetpoint(), intake.backSensor.negate())))
        .withName("Eject")
        .until(ejector.backSensor.negate())
        .withName("Eject")
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  private Command intake(DriveSetpoints point) {
    return elevator
        .goToSetpoint(ElevatorSetpoints.BASE, intake.backSensor.negate())
        .until(elevator.atSetpoint)
        .andThen(drive.autoAlign(() -> point.getPose()).until(intake.frontSensor))
        .andThen(Commands.waitSeconds(1.5))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }
}
