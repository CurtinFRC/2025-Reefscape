package org.curtinfrc.frc2025;

import static org.curtinfrc.frc2025.subsystems.drive.DriveConstants.DriveSetpoints.*;
import static org.curtinfrc.frc2025.subsystems.vision.VisionConstants.*;

import choreo.auto.AutoFactory;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.List;
// import org.curtinfrc.frc2025.Autos.AlgaePoppedStates;
// import org.curtinfrc.frc2025.Autos.AlgaePoppedStates.AlgaeLocations;
import org.curtinfrc.frc2025.Constants.Mode;
import org.curtinfrc.frc2025.Constants.Setpoint;
import org.curtinfrc.frc2025.generated.CompTunerConstants;
import org.curtinfrc.frc2025.generated.DevTunerConstants;
import org.curtinfrc.frc2025.subsystems.climber.Climber;
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
// import org.curtinfrc.frc2025.subsystems.popper.Popper;
// import org.curtinfrc.frc2025.subsystems.popper.PopperIO;
// import org.curtinfrc.frc2025.subsystems.popper.PopperIOKraken;
import org.curtinfrc.frc2025.subsystems.vision.Vision;
import org.curtinfrc.frc2025.subsystems.vision.VisionIO;
import org.curtinfrc.frc2025.subsystems.vision.VisionIOLimelight;
import org.curtinfrc.frc2025.subsystems.vision.VisionIOPhotonVision;
import org.curtinfrc.frc2025.subsystems.vision.VisionIOPhotonVisionSim;
import org.curtinfrc.frc2025.util.AutoChooser;
import org.curtinfrc.frc2025.util.ButtonBoard;
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
  private Elevator elevator;
  private Ejector ejector;
  private Climber climber;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final ButtonBoard board = new ButtonBoard(1);

  // Auto stuff
  private final AutoChooser autoChooser;
  private final AutoFactory factory;
  // private final Autos autos;

  private final List<Pose2d> leftSetpoints;
  private final List<Pose2d> rightSetpoints;

  @AutoLogOutput(key = "Robot/Overridden")
  private boolean overridden = false;

  @AutoLogOutput(key = "Robot/Overide")
  private final Trigger override = new Trigger(() -> overridden);

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncommitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    switch (Constants.getMode()) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    SignalLogger.start();
    SignalLogger.setPath("/U/logs");
    Logger.registerURCL(URCL.startExternal());
    // Start AdvantageKit logger
    Logger.start();

    DriverStation.waitForDsConnection(60);

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
                  new VisionIOPhotonVision(camera0Name, robotToCamera0),
                  new VisionIOPhotonVision(camera1Name, robotToCamera1),
                  new VisionIOPhotonVision(camera2Name, robotToCamera2),
                  new VisionIOPhotonVision(camera3Name, robotToCamera3));
          elevator = new Elevator(new ElevatorIOComp());
          intake = new Intake(new IntakeIOComp());
          ejector = new Ejector(new EjectorIOComp());
          //   popper = new Popper(new PopperIOKraken());
          climber = new Climber(new ClimberIOComp());
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
                  new VisionIO() {},
                  new VisionIOLimelight(camera1Name, drive::getRotation),
                  new VisionIOLimelight(camera2Name, drive::getRotation),
                  new VisionIO() {});
          elevator = new Elevator(new ElevatorIO() {});
          intake = new Intake(new IntakeIO() {});
          ejector = new Ejector(new EjectorIO() {});
          //   popper = new Popper(new PopperIO() {});
          climber = new Climber(new ClimberIO() {});
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
                  new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                  new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose),
                  new VisionIOPhotonVisionSim(camera2Name, robotToCamera2, drive::getPose),
                  new VisionIOPhotonVisionSim(camera3Name, robotToCamera3, drive::getPose));

          elevator = new Elevator(new ElevatorIOSim());
          intake = new Intake(new IntakeIOSim());
          ejector = new Ejector(new EjectorIOSim());
          //   popper = new Popper(new PopperIO() {});
          climber = new Climber(new ClimberIOSim());
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
              new VisionIO() {},
              new VisionIO() {},
              new VisionIO() {},
              new VisionIO() {});

      elevator = new Elevator(new ElevatorIO() {});
      intake = new Intake(new IntakeIO() {});
      ejector = new Ejector(new EjectorIO() {});
      //   popper = new Popper(new PopperIO() {});
      climber = new Climber(new ClimberIO() {});
    }

    leftSetpoints =
        List.of(A.getPose(), C.getPose(), E.getPose(), G.getPose(), I.getPose(), K.getPose());

    rightSetpoints =
        List.of(B.getPose(), D.getPose(), F.getPose(), H.getPose(), J.getPose(), L.getPose());

    PortForwarder.add(5820, "limelight-3.local", 1181);
    PortForwarder.add(5830, "limelight-3g.local", 1181);
    PortForwarder.add(5821, "limelight-3.local", 5800);
    PortForwarder.add(5831, "limelight-3g.local", 5800);

    autoChooser = new AutoChooser("Auto Chooser");
    factory =
        new AutoFactory(
            drive::getPose,
            drive::setPose,
            drive::followTrajectory,
            true,
            drive,
            drive::logTrajectory);

    // autos = new Autos(drive, elevator, popper, ejector, intake);

    // autoChooser.addCmd("Basic Auto", () -> autos.basicAuto());

    autoChooser.addCmd("Test Path", () -> factory.trajectoryCmd("Test Path"));
    autoChooser.addCmd("One Piece", this::onePiece);
    autoChooser.addCmd("Test Auto", this::testAuto);
    autoChooser.addCmd("Three Coral Right", this::threeCoralRight);
    autoChooser.addCmd("Three Coral Left", this::threeCoralLeft);

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
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    drive
        .atSetpoint
        .and(elevator.atSetpoint)
        .whileTrue(ejector.eject(15).until(ejector.backSensor.negate()));

    controller
        .rightBumper()
        .or(controller.leftBumper())
        .whileTrue(
            elevator
                .goToSetpoint(ElevatorSetpoints.L2, intake.backSensor.negate())
                .until(ejector.backSensor.negate()));
    controller
        .rightTrigger()
        .or(controller.leftTrigger())
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
                    () -> -controller.getLeftY(),
                    () -> -controller.getLeftX(),
                    () -> -controller.getRightX())
                .until(ejector.backSensor.negate()));

    controller
        .leftBumper()
        .or(controller.leftTrigger())
        .and(override.negate())
        .whileTrue(
            drive
                .autoAlignWithOverride(
                    () -> DriveSetpoints.closest(drive::getPose, leftSetpoints),
                    () -> -controller.getLeftY(),
                    () -> -controller.getLeftX(),
                    () -> -controller.getRightX())
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
    // popper.setDefaultCommand(popper.stop());
    elevator.setDefaultCommand(
        elevator
            .goToSetpoint(ElevatorSetpoints.BASE, intake.backSensor.negate())
            .withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    climber.setDefaultCommand(climber.stop());

    controller.rightStick().whileTrue(ejector.eject(30));

    // ejector.backSensor.negate().whileTrue(elevator.goToSetpoint(ElevatorSetpoints.BASE));
    intake
        .backSensor
        .and(elevator.isNotAtCollect.negate())
        .and(elevator.atSetpoint)
        .whileTrue(ejector.eject(8));

    intake.backSensor.negate().and(ejector.frontSensor).whileTrue(ejector.stop());

    intake
        .backSensor
        .negate()
        .and(ejector.frontSensor.negate())
        .and(ejector.backSensor)
        .and(elevator.isNotAtCollect.negate())
        .whileTrue(
            ejector
                .eject(-1)
                .until(ejector.frontSensor)
                .andThen(ejector.stop())
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf));

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

    controller.x().onTrue(Commands.runOnce(() -> overridden = !overridden));

    new Trigger(this::isEnabled).onTrue(climber.disengage());
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Switch thread to high priority to improve loop timing
    Threads.setCurrentThreadPriority(true, 99);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Runs virtual subsystems
    VirtualSubsystem.periodicAll();

    // Runs LoggedNetork items
    autoChooser.periodic();

    if (drive.getCurrentCommand() != null) {
      Logger.recordOutput("Drive/Command", drive.getCurrentCommand().getName());
    }
    if (elevator.getCurrentCommand() != null) {
      Logger.recordOutput("Elevator/Command", elevator.getCurrentCommand().getName());
    }
    if (ejector.getCurrentCommand() != null) {
      Logger.recordOutput("Ejector/Command", ejector.getCurrentCommand().getName());
    }

    // Return to normal thread priority
    Threads.setCurrentThreadPriority(false, 10);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

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
        .autoAlign(() -> DriveSetpoints.C)
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
        .autoAlign(() -> point.driveSetpoint())
        .until(drive.atSetpoint)
        .andThen(
            Commands.parallel(
                drive.autoAlign(() -> point.driveSetpoint()),
                elevator.goToSetpoint(point.elevatorSetpoint(), intake.backSensor.negate())))
        .withName("firststep")
        .until(elevator.atSetpoint)
        .withName("GetToAutoPosition")
        .andThen(
            Commands.parallel(
                drive.autoAlign(() -> point.driveSetpoint()),
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
        .andThen(drive.autoAlign(() -> point).until(intake.frontSensor))
        .andThen(Commands.waitSeconds(1.5))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }
}
