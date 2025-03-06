package org.curtinfrc.frc2025.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static org.curtinfrc.frc2025.subsystems.drive.DriveConstants.*;
import static org.curtinfrc.frc2025.subsystems.vision.VisionConstants.aprilTagLayout;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.curtinfrc.frc2025.Constants;
import org.curtinfrc.frc2025.Constants.Mode;
import org.curtinfrc.frc2025.generated.CompTunerConstants;
import org.curtinfrc.frc2025.subsystems.drive.DriveConstants.DriveSetpoints;
import org.curtinfrc.frc2025.util.RepulsorFieldPlanner;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysIdTranslation;
  private final SysIdRoutine sysIdSteer;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d.kZero);

  private double p = 3.5;
  private double d = 0;
  private double i = 0;

  private final PIDController xController = new PIDController(4.5, 0.0, 0);
  private final PIDController yController = new PIDController(4.5, 0.0, 0);
  private final PIDController headingController = new PIDController(p, i, d);

  private final PIDController xSetpointController = new PIDController(0, 0.0, 0);
  private final PIDController ySetpointController = new PIDController(0, 0.0, 0);

  public Trigger atSetpointPose =
      new Trigger(() -> xSetpointController.atSetpoint() && ySetpointController.atSetpoint());

  public DriveSetpoints setpoint = DriveSetpoints.A;

  @AutoLogOutput(key = "Drive/AngleDiff")
  private double a() {
    return Math.abs(
            getPose().getRotation().getDegrees() - setpoint.getPose().getRotation().getDegrees())
        % 360;
  }

  @AutoLogOutput(key = "Drive/xDiff")
  private double x() {
    return Math.abs(getPose().getX() - setpoint.getPose().getX());
  }

  @AutoLogOutput(key = "Drive/yDiff")
  private double y() {
    return Math.abs(getPose().getY() - setpoint.getPose().getY());
  }

  @AutoLogOutput(key = "Drive/AtSetpoint")
  public Trigger atSetpoint = new Trigger(() -> x() <= 0.02 && y() <= 0.02 && a() <= 1);

  @AutoLogOutput(key = "Drive/AlmostAtSetpoint")
  public Trigger almostAtSetpoint =
      new Trigger(
          () -> {
            return getPose().minus(setpoint.getPose()).getTranslation().getNorm() < 1;
          });

  RepulsorFieldPlanner repulsorFieldPlanner = new RepulsorFieldPlanner();

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, CompTunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, CompTunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, CompTunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, CompTunerConstants.BackRight);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // Configure SysId
    sysIdTranslation =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Second),
                Volts.of(2.5),
                Seconds.of(7.5),
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runTranslationCharacterization(voltage.in(Volts)), null, this));

    // Configure SysId
    sysIdSteer =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runSteerCharacterization(voltage.in(Volts)), null, this));

    headingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected && i >= 0 && i < gyroInputs.odometryYawPositions.length) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Fallback to calculating the angle delta
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    Logger.recordOutput("Drive/xPID/setpoint", xController.getSetpoint());
    Logger.recordOutput("Drive/xPID/error", xController.getError());
    Logger.recordOutput("Drive/xPID/atSetpoint", xController.atSetpoint());

    Logger.recordOutput("Drive/yPID/setpoint", yController.getSetpoint());
    Logger.recordOutput("Drive/yPID/error", yController.getError());
    Logger.recordOutput("Drive/yPID/atSetpoint", yController.atSetpoint());

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.getMode() != Mode.SIM);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  private void runVelocity(ChassisSpeeds speeds) {
    SwerveModuleState[] setpointStates =
        kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(speeds, 0.02));
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, CompTunerConstants.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", speeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  private void runCurrentStates(SwerveModuleState[] states) {
    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/TorqueSetpoints", states);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpointTorque(states[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", states);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  private void runSteerCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runSteerCharacterization(output);
    }
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdSteerQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runSteerCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysIdSteer.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdSteerDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runSteerCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysIdSteer.dynamic(direction));
  }

  /** Runs the drive in a straight line with the specified drive output. */
  private void runTranslationCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runTranslationCharacterization(output);
    }
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdTranslationQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runTranslationCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysIdTranslation.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdTranslationDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runTranslationCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysIdTranslation.dynamic(direction));
  }

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public Command joystickDrive(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
    return run(() -> {
          double xSpeed = xSupplier.getAsDouble();

          double ySpeed = ySupplier.getAsDouble();

          double omegaSpeed = omegaSupplier.getAsDouble();

          Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSpeed, ySpeed);

          double omega = MathUtil.applyDeadband(omegaSpeed, DEADBAND);

          omega = Math.copySign(omega * omega, omega);

          Logger.recordOutput("Drive/OmegaUnlimited", omega * getMaxAngularSpeedRadPerSec());

          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * getMaxLinearSpeedMetersPerSec(),
                  omega * getMaxAngularSpeedRadPerSec());

          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;

          runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds, isFlipped ? getRotation().plus(Rotation2d.kPi) : getRotation()));
        })
        .withName("JoystickDrive");
  }
  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public Command joystickDriveAtAngle(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return run(() -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Calculate angular speed
          double omega =
              angleController.calculate(
                  getRotation().getRadians(), rotationSupplier.get().getRadians());

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * getMaxLinearSpeedMetersPerSec(),
                  omega);
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds, isFlipped ? getRotation().plus(Rotation2d.kPi) : getRotation()));
        })
        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(getRotation().getRadians()));
  }

  /** Follows the provided swerve sample. */
  public void followTrajectory(SwerveSample sample) {
    // Get the current pose of the robot
    Pose2d pose = getPose();
    Logger.recordOutput("Odometry/TrajectorySetpoint", pose);
    Logger.recordOutput("Drive/PID/error", headingController.getError());
    // Logger.recordOutput("Drive/PID/out", out);
    Logger.recordOutput("Drive/sample", sample);

    var err = new Transform2d(sample.x - pose.getX(), sample.y - pose.getY(), new Rotation2d());
    var dist = Math.hypot(err.getX(), err.getY());
    Logger.recordOutput("Drive/dist", dist);

    var target_pose =
        (DriverStation.getAlliance().get() == Alliance.Blue
            ? new Pose2d(4.476, 4.026, new Rotation2d())
            : new Pose2d(13.071, 4.026, new Rotation2d()));
    var transform = target_pose.relativeTo(pose).rotateBy(pose.getRotation());
    Logger.recordOutput("Drive/targetpose", target_pose);
    Logger.recordOutput("Drive/transform", transform);
    Logger.recordOutput("Drive/theta", Math.atan2(transform.getY(), transform.getX()));
    Logger.recordOutput(
        "Drive/projected",
        new Pose2d(
            pose.getX(),
            pose.getY(),
            new Rotation2d(Math.atan2(transform.getY(), transform.getX()))));
    // Generate the next speeds for the robot
    xController.setSetpoint(sample.x);
    yController.setSetpoint(sample.y);
    headingController.setSetpoint(sample.heading);
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    ChassisSpeeds speeds;
    if (isFlipped) {
      speeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              sample.vx + (sample.vx != 0 ? 0 : xController.calculate(pose.getX(), sample.x)),
              sample.vy + (sample.vy != 0 ? 0 : yController.calculate(pose.getY(), sample.y)),
              dist < 0.5
                  ? headingController.calculate(pose.getRotation().getRadians(), sample.heading)
                  : headingController.calculate(
                      pose.getRotation().getRadians(),
                      Math.atan2(transform.getY(), transform.getX())),
              getRotation()); // Apply the generated speeds
    } else {
      speeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              sample.vx + (sample.vx != 0 ? 0 : xController.calculate(pose.getX(), sample.x)),
              sample.vy + (sample.vy != 0 ? 0 : yController.calculate(pose.getY(), sample.y)),
              dist < 0.5
                  ? -headingController.calculate(pose.getRotation().getRadians(), sample.heading)
                  : -headingController.calculate(
                      pose.getRotation().getRadians(),
                      Math.atan2(transform.getY(), transform.getX())),
              getRotation()); // Apply the generated speeds
    }

    Logger.recordOutput("Drive/ChassisSpeeds1", speeds);
    runVelocity(speeds);
  }

  public void followTrajectoryVelocity(SwerveSample sample) {
    var xController = new PIDController(10, 0, 0);
    var yController = new PIDController(10, 0, 0);
    var rotationController = new PIDController(7.5, 0, 0);
    Logger.recordOutput("Odometry/Sample", sample);
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    Rotation2d rotation = isFlipped ? getRotation().plus(Rotation2d.kPi) : getRotation();

    // Generate the next speeds for the robot
    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            sample.vx + xController.calculate(getPose().getX(), sample.x),
            sample.vy + yController.calculate(getPose().getY(), sample.y),
            sample.omega + rotationController.calculate(getRotation().getRadians(), sample.heading),
            rotation); // Apply the generated speeds

    runVelocity(speeds);
  }

  public void followTrajectoryTorque(SwerveSample sample) {
    Logger.recordOutput("Odometry/Sample", sample);
    var states = new SwerveModuleState[4];

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    Rotation2d rotation = isFlipped ? getRotation().plus(Rotation2d.kPi) : getRotation();
    Logger.recordOutput("Drive/Kt", kT);

    for (var i = 0; i < 4; i++) {
      states[i] = new SwerveModuleState();
      var state = states[i];
      Translation2d f = new Translation2d(sample.moduleForcesX()[i], sample.moduleForcesY()[i]);
      Translation2d f_fieldRelative = f.rotateBy(rotation);

      // Let torque be τ, current be i, Kt be the motor torque constant, r be the wheel radius
      // vector,
      // and F be the module force vector.
      // τ=Kti
      // τ=r×F
      // Kti=r×F
      // i=(r×F)/Kt
      Vector<N3> F = VecBuilder.fill(f_fieldRelative.getX(), f_fieldRelative.getY(), 0);
      Vector<N3> radius = VecBuilder.fill(0, 0, Units.inchesToMeters(-2));
      var current = Vector.cross(radius, F).div(kT);
      state.speedMetersPerSecond = Math.hypot(current.get(0), current.get(1));
      state.angle = Rotation2d.fromRadians(Math.atan2(current.get(1), current.get(0)));
    }

    runCurrentStates(states);
  }

  public void logTrajectory(Trajectory<SwerveSample> traj, boolean isFinished) {
    SwerveSample[] trajarray = new SwerveSample[0];
    boolean flip =
        DriverStation.isDSAttached() && DriverStation.getAlliance().get() != Alliance.Blue;
    Logger.recordOutput(
        "Odometry/Trajectory",
        flip ? traj.flipped().samples().toArray(trajarray) : traj.samples().toArray(trajarray));
    Logger.recordOutput("Odometry/TrajectoryFinished", isFinished);
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public Command feedforwardCharacterization() {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        run(() -> {
              runTranslationCharacterization(0.0);
            })
            .withTimeout(FF_START_DELAY),

        // Start timer
        runOnce(timer::restart),

        // Accelerate and gather data
        run(() -> {
              double voltage = timer.get() * FF_RAMP_RATE;
              runTranslationCharacterization(voltage);
              velocitySamples.add(getFFCharacterizationVelocity());
              voltageSamples.add(voltage);
            })

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public Command wheelRadiusCharacterization() {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            run(() -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                })
                .withTimeout(5),

            // Measurement sequence
            Commands.sequence(
                // Wait for modules to fully orient before starting measurement
                Commands.waitSeconds(1.0),

                // Record starting measurement
                runOnce(
                    () -> {
                      state.positions = getWheelRadiusCharacterizationPositions();
                      state.lastAngle = getRotation();
                      state.gyroDelta = 0.0;
                    }),

                // Update gyro delta
                run(() -> {
                      var rotation = getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      double[] positions = getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      state.lastAngle = rotation;
                      double wheelRadius = (state.gyroDelta * DRIVE_BASE_RADIUS) / wheelDelta;
                      Logger.recordOutput("WheelRadius", wheelRadius);
                    })

                    // When cancelled, calculate and print results
                    .finallyDo(
                        () -> {
                          double[] positions = getWheelRadiusCharacterizationPositions();
                          double wheelDelta = 0.0;
                          for (int i = 0; i < 4; i++) {
                            wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                          }
                          double wheelRadius = (state.gyroDelta * DRIVE_BASE_RADIUS) / wheelDelta;

                          NumberFormat formatter = new DecimalFormat("#0.000");
                          System.out.println(
                              "********** Wheel Radius Characterization Results **********");
                          System.out.println(
                              "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                          System.out.println(
                              "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                          System.out.println(
                              "\tWheel Radius: "
                                  + formatter.format(wheelRadius)
                                  + " meters, "
                                  + formatter.format(Units.metersToInches(wheelRadius))
                                  + " inches");
                        }))));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return CompTunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(
          CompTunerConstants.FrontLeft.LocationX, CompTunerConstants.FrontLeft.LocationY),
      new Translation2d(
          CompTunerConstants.FrontRight.LocationX, CompTunerConstants.FrontRight.LocationY),
      new Translation2d(
          CompTunerConstants.BackLeft.LocationX, CompTunerConstants.BackLeft.LocationY),
      new Translation2d(
          CompTunerConstants.BackRight.LocationX, CompTunerConstants.BackRight.LocationY)
    };
  }

  public Command autoAlignWithOverride(
      Supplier<DriveSetpoints> _setpoint,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return autoAlign(
        _setpoint, Optional.of(xSupplier), Optional.of(ySupplier), Optional.of(omegaSupplier));
  }

  public Command autoAlign(
      Supplier<DriveSetpoints> _setpoint,
      Optional<DoubleSupplier> xSupplier,
      Optional<DoubleSupplier> ySupplier,
      Optional<DoubleSupplier> omegaSupplier) {
    return run(() -> {
          if (xSupplier.isPresent() && ySupplier.isPresent() && omegaSupplier.isPresent()) {
            if (Math.abs(xSupplier.get().getAsDouble()) > 0.05
                || Math.abs(ySupplier.get().getAsDouble()) > 0.05
                || Math.abs(omegaSupplier.get().getAsDouble()) > 0.05) {
              joystickDrive(xSupplier.get(), ySupplier.get(), omegaSupplier.get()).execute();
              return;
            }
          }

          this.setpoint = _setpoint.get();
          Logger.recordOutput("Drive/Setpoint", this.setpoint.getPose());

          repulsorFieldPlanner.setGoal(this.setpoint.getPose().getTranslation());

          var robotPose = getPose();
          SwerveSample cmd =
              repulsorFieldPlanner.getCmd(
                  robotPose,
                  getChassisSpeeds(),
                  CompTunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
                  true);

          // Apply the trajectory with rotation adjustment
          SwerveSample adjustedSample =
              new SwerveSample(
                  cmd.t,
                  cmd.x,
                  cmd.y,
                  this.setpoint.getPose().getRotation().getRadians(),
                  cmd.vx,
                  cmd.vy,
                  0,
                  cmd.ax,
                  cmd.ay,
                  cmd.alpha,
                  cmd.moduleForcesX(),
                  cmd.moduleForcesY());

          // Apply the adjusted sample
          followTrajectory(adjustedSample);
        })
        .withName("AutoAlign");
  }

  public Pose3d findClosestTag(List<AprilTag> tags) {
    Transform2d lowestTransform = null;
    int closestTagId = 99;
    for (var tag : tags) {
      var transform = getPose().minus(tag.pose.toPose2d());
      if (lowestTransform == null) {
        lowestTransform = transform;
        closestTagId = tag.ID;
        break;
      }
      if (lowestTransform.getTranslation().getNorm() > transform.getTranslation().getNorm()) {
        lowestTransform = transform;
        closestTagId = tag.ID;
      }
    }
    return aprilTagLayout.getTagPose(closestTagId).get();
  }
}
