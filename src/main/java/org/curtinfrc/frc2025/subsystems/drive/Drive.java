package org.curtinfrc.frc2025.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static org.curtinfrc.frc2025.subsystems.drive.DriveConstants.*;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
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
import edu.wpi.first.math.system.plant.DCMotor;
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
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.curtinfrc.frc2025.Constants;
import org.curtinfrc.frc2025.Constants.Mode;
import org.curtinfrc.frc2025.generated.CompTunerConstants;
import org.curtinfrc.frc2025.subsystems.drive.DriveConstants.DriveSetpoints;
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

  private final PIDController xController = new PIDController(3.5, 0, 0);
  private final PIDController yController = new PIDController(3.5, 0, 0);
  private final PIDController headingController = new PIDController(3.5, 0, 0);

  private final PIDController xFollower = new PIDController(1, 0, 0);
  private final PIDController yFollower = new PIDController(1, 0, 0);
  private final PIDController headingFollower = new PIDController(1.3, 0, 0);

  @AutoLogOutput(key = "Drive/Setpoint")
  public DriveSetpoints setpoint = DriveSetpoints.A;

  @AutoLogOutput(key = "Drive/AtSetpoint")
  public Trigger atSetpoint =
      new Trigger(
          () ->
              xController.atSetpoint()
                  && yController.atSetpoint()
                  && headingController.atSetpoint());

  @AutoLogOutput(key = "Drive/AlmostAtSetpoint")
  public Trigger almostAtSetpoint =
      new Trigger(
          () -> {
            return getPose().minus(setpoint.getPose()).getTranslation().getNorm() < 1;
          });

  private final RepulsorFieldPlanner repulsorFieldPlanner = new RepulsorFieldPlanner();

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

    xController.setTolerance(0.02);
    yController.setTolerance(0.02);
    headingController.setTolerance(0.02);
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    headingFollower.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void followTrajectory(SwerveSample sample) {
    Logger.recordOutput("Odometry/Sample", sample);
    Logger.recordOutput("Odometry/TargetPose", sample.getPose());
    var pose = getPose();
    var feedforwards = new double[4];
    var forcesX = sample.moduleForcesX();
    var forcesY = sample.moduleForcesY();
    // Let torque be τ, current be i, Kt be the motor torque constant, r be the wheel radius vector,
    // and F be the module force vector
    // τ=Kt
    // τ=r×F
    // K_ti=r×F
    // i =(r×F)/K_t
    var kT = DCMotor.getKrakenX60Foc(1).KtNMPerAmp * 5.99;
    var wheelRadius = VecBuilder.fill(0, 0, 0.0508);
    for (var i = 0; i < forcesX.length; i++) {
      var translation = new Translation2d(forcesX[i], forcesY[i]);
      var rotated = translation.rotateBy(getRotation().unaryMinus());
      var force = VecBuilder.fill(rotated.getX(), rotated.getY(), 0);
      feedforwards[i] = Vector.cross(wheelRadius, force).div(kT).norm();
    }

    var speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xFollower.calculate(pose.getX(), sample.x) + sample.vx,
            yFollower.calculate(pose.getY(), sample.y) + sample.vy,
            headingFollower.calculate(getRotation().getRadians(), sample.heading) + sample.omega,
            getRotation());

    runVelocity(speeds, new double[4]);
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

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.getMode() != Mode.SIM);

    Logger.recordOutput("xPidAtSetpoint", xController.atSetpoint());
    Logger.recordOutput("yPidAtSetpoint", yController.atSetpoint());
    Logger.recordOutput("omegaPidAtSetpoint", headingController.atSetpoint());
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  private void runVelocity(ChassisSpeeds speeds, double[] feedforwardAmps) {
    SwerveModuleState[] setpointStates =
        kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(speeds, 0.02));
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, CompTunerConstants.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", speeds);
    Logger.recordOutput("SwerveChassisSpeeds/FeedForwardsAmps", feedforwardAmps);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i], feedforwardAmps[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
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
                  speeds, isFlipped ? getRotation().plus(Rotation2d.kPi) : getRotation()),
              new double[4]);
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
                  speeds, isFlipped ? getRotation().plus(Rotation2d.kPi) : getRotation()),
              new double[4]);
        })
        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(getRotation().getRadians()));
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
                  runVelocity(new ChassisSpeeds(0.0, 0.0, speed), new double[4]);
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
    return run(
        () -> {
          this.setpoint = _setpoint.get();
          if (Math.abs(xSupplier.getAsDouble()) > 0.05
              || Math.abs(ySupplier.getAsDouble()) > 0.05
              || Math.abs(omegaSupplier.getAsDouble()) > 0.05) {
            joystickDrive(xSupplier, ySupplier, omegaSupplier).execute();
            return;
          }
          autoAlign(_setpoint.get().getPose());
        });
  }

  private void autoAlign(Pose2d _setpoint) {
    Logger.recordOutput("Drive/AutoAlignSetpoint", _setpoint);
    var robotPose = getPose();

    var omega =
        headingController.calculate(
            getRotation().getRadians(), _setpoint.getRotation().getRadians());
    var x = xController.calculate(robotPose.getX(), _setpoint.getX());
    var y = yController.calculate(robotPose.getY(), _setpoint.getY());
    runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(x, y, omega, getRotation()), new double[4]);
  }

  public Command autoAlign(Supplier<Pose2d> _setpoint) {
    return run(() -> autoAlign(_setpoint.get())).withName("AutoAlign");
  }
}
