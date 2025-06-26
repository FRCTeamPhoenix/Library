// Copyright (c) 2025 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc.subsystems.drive;

import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team2342.frc.Constants.DriveConstants;
import org.team2342.lib.logging.ExecutionLogger;
import org.team2342.lib.util.LocalADStarAK;
import org.team2342.lib.util.SwerveSetpointGenerator;
import org.team2342.lib.util.SwerveSetpointGenerator.ModuleLimits;
import org.team2342.lib.util.SwerveSetpointGenerator.SwerveSetpoint;

public class Drive extends SubsystemBase {
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  // Module Order: FL, FR, BL, BR
  private final Module[] modules = new Module[4];

  private final Field2d dashboardField = new Field2d();
  private final Alert gyroAlert =
      new Alert("Gyro disconnected, using fallback!", AlertType.kWarning);

  @Getter private final RobotConfig pathplannerConfig;
  private final SysIdRoutine sysId;

  protected static final Lock odometryLock = new ReentrantLock();
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();

  private Rotation2d visionGyroOffset = new Rotation2d();
  private boolean offsetDone = false;

  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());
  private SwerveDrivePoseEstimator rawOdometry =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  private final SwerveSetpointGenerator setpointGenerator;
  private SwerveSetpoint previousSetpoint;
  private ModuleLimits moduleLimits;

  @Getter private double maxLinearSpeedMetersPerSec = DriveConstants.MAX_LINEAR_SPEED;
  @Getter private double maxAngularSpeedRadPerSec = DriveConstants.MAX_ANGULAR_SPEED;
  @Getter private double driveBaseRadius = DriveConstants.DRIVE_BASE_RADIUS;

  public Drive(GyroIO gyro, ModuleIO fl, ModuleIO fr, ModuleIO bl, ModuleIO br) {
    this.gyroIO = gyro;
    modules[0] = new Module(fl, 0);
    modules[1] = new Module(fr, 1);
    modules[2] = new Module(bl, 2);
    modules[3] = new Module(br, 3);

    // Start odometry thread
    PhoenixOdometry.getInstance().start();

    // Create PathPlanner config
    pathplannerConfig =
        new RobotConfig(
            DriveConstants.ROBOT_MASS_KG,
            DriveConstants.ROBOT_MOI,
            new ModuleConfig(
                DriveConstants.WHEEL_RADIUS,
                DriveConstants.MAX_LINEAR_SPEED,
                DriveConstants.WHEEL_COF,
                DCMotor.getKrakenX60(1).withReduction(DriveConstants.DRIVE_GEARING),
                DriveConstants.SLIP_CURRENT_LIMIT,
                1),
            getModuleTranslations());

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(6.0, 0.0, 0.0), new PIDConstants(8.0, 0.0, 0.1)),
        pathplannerConfig,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);

    // Logging callbacks for PathPlanner
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));

    // Setup setpoint generator
    setpointGenerator = new SwerveSetpointGenerator(kinematics);
    previousSetpoint = new SwerveSetpoint(getChassisSpeeds(), getModuleStates());
    moduleLimits =
        new ModuleLimits(
            maxLinearSpeedMetersPerSec,
            DriveConstants.MAX_LINEAR_ACCELERATION,
            DriveConstants.MAX_MODULE_VELOCITY_RAD);

    // Put swerve widget on the dashboard
    SmartDashboard.putData("DashboardSwerve", this);
    SmartDashboard.putData("DashboardField", dashboardField);
  }

  @Override
  public void periodic() {
    odometryLock.lock();
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
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
      rawOdometry.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    gyroAlert.set(!gyroInputs.connected);

    dashboardField.setRobotPose(getPose());

    ExecutionLogger.log("Drive/Periodic");
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Generate new setpoint, using previous setpoint
    previousSetpoint =
        setpointGenerator.generateSetpoint(moduleLimits, previousSetpoint, speeds, 0.02);

    // Log setpoint outputs
    Logger.recordOutput("SwerveStates/Setpoints", previousSetpoint.moduleStates());
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", previousSetpoint.chassisSpeeds());

    for (int i = 0; i < 4; i++) {
      // Run each module to the specified state
      modules[i].runSetpoint(previousSetpoint.moduleStates()[i]);
    }

    // Log mutated states
    Logger.recordOutput("SwerveStates/SetpointsOptimized", previousSetpoint.moduleStates());
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
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
  @AutoLogOutput(key = "Odometry/Fused")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Raw")
  public Pose2d getRawOdometryPose() {
    return rawOdometry.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  public Rotation2d getVisionGyroHeading() {
    if (offsetDone) {
      Logger.recordOutput("Vision/Constrained/Heading", rawGyroRotation.minus(visionGyroOffset));
      return rawGyroRotation.minus(visionGyroOffset);
    }
    return null;
  }

  public void setVisionGyroOffset() {
    visionGyroOffset = getRotation().minus(rawGyroRotation);
    offsetDone = true;
    Logger.recordOutput("Vision/Constrained/Offset", visionGyroOffset);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(DriveConstants.TRACK_WIDTH_X / 2.0, DriveConstants.TRACK_WIDTH_Y / 2.0),
      new Translation2d(DriveConstants.TRACK_WIDTH_X / 2.0, -DriveConstants.TRACK_WIDTH_Y / 2.0),
      new Translation2d(-DriveConstants.TRACK_WIDTH_X / 2.0, DriveConstants.TRACK_WIDTH_Y / 2.0),
      new Translation2d(-DriveConstants.TRACK_WIDTH_X / 2.0, -DriveConstants.TRACK_WIDTH_Y / 2.0)
    };
  }

  /** Print the absolute angle of each module (no offset). */
  public void printModuleAbsoluteAngles() {
    for (var module : modules) {
      System.out.println(module.getAbsoluteAngle());
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("SwerveDrive");

    builder.addDoubleProperty("Front Left Angle", () -> modules[0].getAngle().getRadians(), null);
    builder.addDoubleProperty(
        "Front Left Velocity", () -> modules[0].getVelocityMetersPerSec(), null);

    builder.addDoubleProperty("Front Right Angle", () -> modules[1].getAngle().getRadians(), null);
    builder.addDoubleProperty(
        "Front Right Velocity", () -> modules[1].getVelocityMetersPerSec(), null);

    builder.addDoubleProperty("Back Left Angle", () -> modules[2].getAngle().getRadians(), null);
    builder.addDoubleProperty(
        "Back Left Velocity", () -> modules[2].getVelocityMetersPerSec(), null);

    builder.addDoubleProperty("Back Right Angle", () -> modules[3].getAngle().getRadians(), null);
    builder.addDoubleProperty(
        "Back Right Velocity", () -> modules[3].getVelocityMetersPerSec(), null);
    builder.addDoubleProperty(
        "Robot Angle",
        () ->
            getRotation().getRadians()
                + (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red ? Math.PI : 0),
        null);
  }
}
