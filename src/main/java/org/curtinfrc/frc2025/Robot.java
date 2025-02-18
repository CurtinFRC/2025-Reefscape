package org.curtinfrc.frc2025;

import static org.curtinfrc.frc2025.subsystems.intake.IntakeConstants.intakeVolts;
import static org.curtinfrc.frc2025.subsystems.vision.VisionConstants.*;

import choreo.auto.AutoFactory;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.Map;
import org.curtinfrc.frc2025.Constants.Mode;
import org.curtinfrc.frc2025.generated.TunerConstants;
import org.curtinfrc.frc2025.subsystems.drive.Drive;
import org.curtinfrc.frc2025.subsystems.drive.DriveConstants.DriveSetpoints;
import org.curtinfrc.frc2025.subsystems.drive.GyroIO;
import org.curtinfrc.frc2025.subsystems.drive.GyroIOPigeon2;
import org.curtinfrc.frc2025.subsystems.drive.GyroIOSim;
import org.curtinfrc.frc2025.subsystems.drive.ModuleIO;
import org.curtinfrc.frc2025.subsystems.drive.ModuleIOSim;
import org.curtinfrc.frc2025.subsystems.drive.ModuleIOTalonFX;
import org.curtinfrc.frc2025.subsystems.ejector.Ejector;
import org.curtinfrc.frc2025.subsystems.ejector.EjectorConstants;
import org.curtinfrc.frc2025.subsystems.ejector.EjectorIO;
import org.curtinfrc.frc2025.subsystems.ejector.EjectorIONEO;
import org.curtinfrc.frc2025.subsystems.ejector.EjectorIOSim;
import org.curtinfrc.frc2025.subsystems.elevator.Elevator;
import org.curtinfrc.frc2025.subsystems.elevator.ElevatorConstants;
import org.curtinfrc.frc2025.subsystems.elevator.ElevatorConstants.ElevatorSetpoints;
import org.curtinfrc.frc2025.subsystems.elevator.ElevatorIO;
import org.curtinfrc.frc2025.subsystems.elevator.ElevatorIONEO;
import org.curtinfrc.frc2025.subsystems.elevator.ElevatorIOSim;
import org.curtinfrc.frc2025.subsystems.intake.Intake;
import org.curtinfrc.frc2025.subsystems.intake.IntakeConstants;
import org.curtinfrc.frc2025.subsystems.intake.IntakeIO;
import org.curtinfrc.frc2025.subsystems.intake.IntakeIONEO;
import org.curtinfrc.frc2025.subsystems.intake.IntakeIOSim;
import org.curtinfrc.frc2025.subsystems.vision.Vision;
import org.curtinfrc.frc2025.subsystems.vision.VisionIO;
import org.curtinfrc.frc2025.subsystems.vision.VisionIOLimelight;
import org.curtinfrc.frc2025.subsystems.vision.VisionIOLimelightGamepiece;
import org.curtinfrc.frc2025.subsystems.vision.VisionIOPhotonVision;
import org.curtinfrc.frc2025.subsystems.vision.VisionIOPhotonVisionSim;
import org.curtinfrc.frc2025.subsystems.vision.VisionIOQuestNav;
import org.curtinfrc.frc2025.util.AutoChooser;
import org.curtinfrc.frc2025.util.ButtonBoard;
import org.curtinfrc.frc2025.util.VirtualSubsystem;
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

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final ButtonBoard board = new ButtonBoard(1);

  // Auto stuff
  private final AutoChooser autoChooser;
  private final AutoFactory autoFactory;
  private final Autos autos;

  // Setpoints
  public static record Setpoint(ElevatorSetpoints elevatorSetpoint, DriveSetpoints driveSetpoint) {
    public static Setpoint NULL() {
      return new Setpoint(ElevatorSetpoints.BASE, DriveSetpoints.NULL);
    }
  }

  private Setpoint currentSetpoint = new Setpoint(ElevatorSetpoints.BASE, DriveSetpoints.NULL);
  private Setpoint nextSetpoint = new Setpoint(ElevatorSetpoints.BASE, DriveSetpoints.NULL);

  // Triggers
  public final Trigger atSetpoint = elevator.atSetpoint.and(drive.atSetpoint);
  // TODO end sensor triggered
  public final Trigger setpointDone =
      atSetpoint.or(new Trigger(() -> currentSetpoint.equals(Setpoint.NULL())));
  public final Trigger hasSetpoint = new Trigger(() -> !currentSetpoint.equals(Setpoint.NULL()));

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
    Logger.registerURCL(
        URCL.startExternal(
            Map.of(
                ElevatorConstants.motorPort,
                "Elevator",
                EjectorConstants.motorId,
                "Ejector",
                IntakeConstants.intakeMotorId,
                "Intake")));
    // Start AdvantageKit logger
    Logger.start();

    DriverStation.waitForDsConnection(60);

    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.robotType) {
        case COMPBOT -> {
          // Real robot, instantiate hardware IO implementationsRobot
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(TunerConstants.FrontLeft),
                  new ModuleIOTalonFX(TunerConstants.FrontRight),
                  new ModuleIOTalonFX(TunerConstants.BackLeft),
                  new ModuleIOTalonFX(TunerConstants.BackRight));
          vision =
              new Vision(
                  drive::addVisionMeasurement,
                  new VisionIOLimelightGamepiece(camera0Name),
                  new VisionIOLimelight(camera1Name, drive::getRotation),
                  new VisionIOQuestNav());
          elevator = new Elevator(new ElevatorIO() {});
          intake = new Intake(new IntakeIONEO());
          ejector = new Ejector(new EjectorIONEO());
        }

        case DEVBOT -> {
          // Real robot, instantiate hardware IO implementationsRobot
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(TunerConstants.FrontLeft),
                  new ModuleIOTalonFX(TunerConstants.FrontRight),
                  new ModuleIOTalonFX(TunerConstants.BackLeft),
                  new ModuleIOTalonFX(TunerConstants.BackRight));
          vision =
              new Vision(
                  drive::addVisionMeasurement,
                  new VisionIOPhotonVision(camera0Name, robotToCamera0),
                  new VisionIOLimelight(camera1Name, drive::getRotation),
                  new VisionIOLimelight(camera2Name, drive::getRotation));
          elevator = new Elevator(new ElevatorIONEO());
          intake = new Intake(new IntakeIONEO());
          ejector = new Ejector(new EjectorIONEO());
        }

        case SIMBOT -> {
          // Sim robot, instantiate physics sim IO implementations
          drive =
              new Drive(
                  new GyroIOSim(
                      () -> drive.getKinematics(),
                      () -> drive.getModuleStates()) {}, // work around crash
                  new ModuleIOSim(TunerConstants.FrontLeft),
                  new ModuleIOSim(TunerConstants.FrontRight),
                  new ModuleIOSim(TunerConstants.BackLeft),
                  new ModuleIOSim(TunerConstants.BackRight));
          vision =
              new Vision(
                  drive::addVisionMeasurement,
                  new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                  new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose),
                  new VisionIO() {});

          elevator = new Elevator(new ElevatorIOSim());
          intake = new Intake(new IntakeIOSim());
          ejector = new Ejector(new EjectorIOSim());
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
              drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {}, new VisionIO() {});

      elevator = new Elevator(new ElevatorIO() {});
      intake = new Intake(new IntakeIO() {});
      ejector = new Ejector(new EjectorIO() {});
    }

    autoFactory =
        new AutoFactory(
            drive::getPose,
            drive::setPose,
            drive::followTrajectoryVelocity,
            true,
            drive,
            drive::logTrajectory);

    autoChooser = new AutoChooser("Auto Chooser");

    autos = new Autos(autoFactory);

    autoChooser.addRoutine("Follow Test Path", () -> autos.followPath("Test Path"));

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

    elevator.isNotAtCollect.and(atSetpoint).whileTrue(ejector.eject(500));

    intake.setDefaultCommand(intake.intake(intakeVolts));
    ejector.setDefaultCommand(
        ejector.stop().withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    elevator.setDefaultCommand(elevator.goToSetpoint(ElevatorSetpoints.BASE));

    intake
        .backSensor
        .and(intake.frontSensor.negate())
        .and(elevator.isNotAtCollect.negate())
        .whileTrue(
            Commands.parallel(intake.intake(intakeVolts), ejector.eject(5)).withName("front"));
    intake
        .backSensor
        .and(intake.frontSensor)
        .and(elevator.isNotAtCollect.negate())
        .whileTrue(
            Commands.parallel(intake.intake(intakeVolts), ejector.eject(5))
                .withName("front and back"));

    intake
        .backSensor
        .and(intake.frontSensor.negate())
        .and(elevator.isNotAtCollect.negate())
        .whileTrue(Commands.parallel(intake.stop(), ejector.stop()).withName("not front and back"));

    intake.frontSensor.whileTrue(elevator.stop());

    controller.b().onTrue(elevator.zero().ignoringDisable(true));

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

    // controller
    //     .rightBumper()
    //     .whileTrue(
    //         Commands.parallel(
    //             elevator.goToSetpoint(ElevatorSetpoints.L2), drive.autoAlign(DriveSetpoints.A)));
    //
    // controller
    //     .leftBumper()
    //     .whileTrue(
    //         Commands.parallel(
    //             elevator.goToSetpoint(ElevatorSetpoints.L2), drive.autoAlign(DriveSetpoints.B)));
    //
    // controller
    //     .rightTrigger()
    //     .whileTrue(
    //         Commands.parallel(
    //             elevator.goToSetpoint(ElevatorSetpoints.L3), drive.autoAlign(DriveSetpoints.A)));
    //
    // controller
    //     .leftTrigger()
    //     .whileTrue(
    //         Commands.parallel(
    //             elevator.goToSetpoint(ElevatorSetpoints.L3), drive.autoAlign(DriveSetpoints.B)));
    //
    // controller.rightStick().whileTrue(drive.autoAlign(DriveSetpoints.RIGHT_HP));
    // controller.leftStick().whileTrue(drive.autoAlign(DriveSetpoints.LEFT_HP));

    setpointDone.onTrue(
        Commands.runOnce(
            () -> {
              currentSetpoint = nextSetpoint;
              nextSetpoint = Setpoint.NULL();
            }));

    hasSetpoint.whileTrue(
        drive
            .autoAlign(currentSetpoint.driveSetpoint())
            .withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    controller
        .rightTrigger()
        .onTrue(
            Commands.runOnce(
                () ->
                    nextSetpoint =
                        new Setpoint(ElevatorSetpoints.L3, nextSetpoint.driveSetpoint())));

    controller
        .leftTrigger()
        .onTrue(
            Commands.runOnce(
                () ->
                    nextSetpoint =
                        new Setpoint(ElevatorSetpoints.L2, nextSetpoint.driveSetpoint())));

    board.left().onTrue(Commands.runOnce(() -> nextSetpoint = new Setpoint(ElevatorSetpoints.BASE, DriveSetpoints.LEFT_HP)));
    board.right().onTrue(Commands.runOnce(() -> nextSetpoint = new Setpoint(ElevatorSetpoints.BASE, DriveSetpoints.RIGHT_HP)));

    board
        .coralAB()
        .and(controller.rightBumper())
        .onTrue(
            Commands.runOnce(
                () -> nextSetpoint = new Setpoint(ElevatorSetpoints.BASE, DriveSetpoints.A)));

    board
        .coralAB()
        .and(controller.leftBumper())
        .onTrue(
            Commands.runOnce(
                () -> nextSetpoint = new Setpoint(ElevatorSetpoints.BASE, DriveSetpoints.B)));

    board
        .coralCD()
        .and(controller.rightBumper())
        .onTrue(
            Commands.runOnce(
                () -> nextSetpoint = new Setpoint(ElevatorSetpoints.BASE, DriveSetpoints.C)));

    board
        .coralCD()
        .and(controller.leftBumper())
        .onTrue(
            Commands.runOnce(
                () -> nextSetpoint = new Setpoint(ElevatorSetpoints.BASE, DriveSetpoints.D)));

    board
        .coralEF()
        .and(controller.rightBumper())
        .onTrue(
            Commands.runOnce(
                () -> nextSetpoint = new Setpoint(ElevatorSetpoints.BASE, DriveSetpoints.E)));

    board
        .coralEF()
        .and(controller.leftBumper())
        .onTrue(
            Commands.runOnce(
                () -> nextSetpoint = new Setpoint(ElevatorSetpoints.BASE, DriveSetpoints.F)));

    board
        .coralGH()
        .and(controller.rightBumper())
        .onTrue(
            Commands.runOnce(
                () -> nextSetpoint = new Setpoint(ElevatorSetpoints.BASE, DriveSetpoints.G)));

    board
        .coralGH()
        .and(controller.leftBumper())
        .onTrue(
            Commands.runOnce(
                () -> nextSetpoint = new Setpoint(ElevatorSetpoints.BASE, DriveSetpoints.H)));

    board
        .coralIJ()
        .and(controller.rightBumper())
        .onTrue(
            Commands.runOnce(
                () -> nextSetpoint = new Setpoint(ElevatorSetpoints.BASE, DriveSetpoints.I)));

    board
        .coralIJ()
        .and(controller.leftBumper())
        .onTrue(
            Commands.runOnce(
                () -> nextSetpoint = new Setpoint(ElevatorSetpoints.BASE, DriveSetpoints.J)));

    board
        .coralKL()
        .and(controller.rightBumper())
        .onTrue(
            Commands.runOnce(
                () -> nextSetpoint = new Setpoint(ElevatorSetpoints.BASE, DriveSetpoints.K)));

    board
        .coralKL()
        .and(controller.leftBumper())
        .onTrue(
            Commands.runOnce(
                () -> nextSetpoint = new Setpoint(ElevatorSetpoints.BASE, DriveSetpoints.L)));
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

    if (ejector.getCurrentCommand() != null) {
      Logger.recordOutput("EjectorCommand", ejector.getCurrentCommand().getName());
    } else {
      Logger.recordOutput("EjectorCommand", "null");
    }

    autoChooser.periodic();

    if (elevator.getCurrentCommand() != null) {
      Logger.recordOutput("ElevatorCommand", elevator.getCurrentCommand().getName());
    } else {
      Logger.recordOutput("ElevatorCommand", "null");
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
  public void teleopPeriodic() {
    if (ejector.getCurrentCommand() != null) {
      Logger.recordOutput("EjectorCommand", ejector.getCurrentCommand().getName());
    } else {
      Logger.recordOutput("EjectorCommand", "null");
    }

    if (intake.getCurrentCommand() != null) {
      Logger.recordOutput("IntakeCommand", intake.getCurrentCommand().getName());
    } else {
      Logger.recordOutput("IntakeCommand", "null");
    }

    if (intake.getCurrentCommand() != null) {
      Logger.recordOutput("DriveCommand", drive.getCurrentCommand().getName());
    } else {
      Logger.recordOutput("DriveCommand", "null");
    }
  }

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
}
