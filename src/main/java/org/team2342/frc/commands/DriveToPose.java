// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc.commands;

import java.util.Optional;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.team2342.frc.subsystems.drive.Drive;
import org.wpilib.command2.Command;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.trajectory.TrapezoidProfile.State;
import org.wpilib.math.util.Units;
import org.wpilib.system.Timer;

public class DriveToPose extends Command {
  public static double MAX_VELOCITY = 4.0;
  public static double MAX_ACCELERATION = 4.0;

  public static double MAX_ANGULAR_VELOCITY = 500.0;
  public static double MAX_ANGULAR_ACCELERATION = 8.0;

  private double driveTolerance = 0.01;
  private double thetaTolerance = Units.degreesToRadians(1.0);
  private double linearFFMinRadius = 0.01;
  private double linearFFMaxRadius = 0.05;
  private double thetaFFMinError = 0.0;
  private double thetaFFMaxError = 0.0;
  private double setpointMinVelocity = -0.5;
  private double minDistanceVelocityCorrection = 0.01;

  private final Drive drive;
  private final Supplier<Pose2d> target;
  private final Supplier<Optional<Double>> omegaOverride;

  private TrapezoidProfile driveProfile =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));
  private final PIDController driveController = new PIDController(1.8, 0.0, 0.0);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          5.0, 0.0, 0.5, new TrapezoidProfile.Constraints(MAX_ANGULAR_VELOCITY, MAX_ACCELERATION));

  private Translation2d lastSetpointTranslation = Translation2d.ZERO;
  private Translation2d lastSetpointVelocity = Translation2d.ZERO;
  private Rotation2d lastGoalRotation = Rotation2d.ZERO;
  private double lastTime = 0.0;
  private double driveErrorAbs = 0.0;
  private double thetaErrorAbs = 0.0;
  @Getter private boolean running = false;

  public DriveToPose(
      Drive drive, Supplier<Pose2d> target, Supplier<Optional<Double>> omegaOverride) {
    this.drive = drive;
    this.target = target;
    this.omegaOverride = omegaOverride;

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drive);
  }

  public DriveToPose(Drive drive, Supplier<Pose2d> target) {
    this(drive, target, () -> Optional.empty());
  }

  public DriveToPose(Drive drive, Pose2d target) {
    this(drive, () -> target, () -> Optional.empty());
  }

  @Override
  public void initialize() {
    Pose2d currentPose = drive.getPose();
    Pose2d targetPose = target.get();
    ChassisVelocities fieldVelocity = drive.getChassisVelocities();
    Translation2d linearFieldVelocity = new Translation2d(fieldVelocity.vx, fieldVelocity.vy);

    driveProfile =
        new TrapezoidProfile(new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));

    driveController.reset();
    thetaController.reset(currentPose.getRotation().getRadians(), fieldVelocity.omega);
    lastSetpointTranslation = currentPose.getTranslation();
    lastSetpointVelocity = linearFieldVelocity;
    lastGoalRotation = targetPose.getRotation();
    lastTime = Timer.getTimestamp();
  }

  @Override
  public void execute() {
    running = true;

    // driveProfile =
    //     new TrapezoidProfile(
    //         new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));
    // thetaController.setConstraints(
    //     new TrapezoidProfile.Constraints(MAX_ANGULAR_VELOCITY, MAX_ACCELERATION));

    // Get current pose and target pose
    Pose2d currentPose = drive.getPose();
    Pose2d targetPose = target.get();

    Pose2d poseError = currentPose.relativeTo(targetPose);
    driveErrorAbs = poseError.getTranslation().getNorm();
    thetaErrorAbs = Math.abs(poseError.getRotation().getRadians());
    double linearFFScaler =
        Math.clamp(
            (driveErrorAbs - linearFFMinRadius) / (linearFFMaxRadius - linearFFMinRadius),
            0.0,
            1.0);
    double thetaFFScaler =
        Math.clamp(
            (Units.radiansToDegrees(thetaErrorAbs) - thetaFFMinError)
                / (thetaFFMaxError - thetaFFMinError),
            0.0,
            1.0);

    // Calculate drive velocity
    // Calculate setpoint velocity towards target pose
    var direction = targetPose.getTranslation().minus(lastSetpointTranslation).toVector();
    double setpointVelocity =
        direction.norm()
                <= minDistanceVelocityCorrection // Don't calculate velocity in direction when
            // really close
            ? lastSetpointVelocity.getNorm()
            : lastSetpointVelocity.toVector().dot(direction) / direction.norm();
    setpointVelocity = Math.max(setpointVelocity, setpointMinVelocity);
    State driveSetpoint =
        driveProfile.calculate(
            0.02,
            new State(
                direction.norm(), -setpointVelocity), // Use negative as profile has zero at target
            new State(0.0, 0.0));
    double driveVelocityScalar =
        driveController.calculate(driveErrorAbs, driveSetpoint.position)
            + driveSetpoint.velocity * linearFFScaler;
    if (driveErrorAbs < driveController.getErrorTolerance()) driveVelocityScalar = 0.0;
    Rotation2d targetToCurrentAngle =
        currentPose
            .getTranslation()
            .minus(targetPose.getTranslation())
            .getAngle()
            .orElse(Rotation2d.ZERO);

    Translation2d driveVelocity = new Translation2d(driveVelocityScalar, targetToCurrentAngle);
    lastSetpointTranslation =
        new Pose2d(targetPose.getTranslation(), targetToCurrentAngle)
            .transformBy(new Transform2d(driveSetpoint.position, 0.0, Rotation2d.ZERO))
            .getTranslation();
    lastSetpointVelocity = new Translation2d(driveSetpoint.velocity, targetToCurrentAngle);

    // Calculate theta speed
    double thetaSetpointVelocity =
        Math.abs((targetPose.getRotation().minus(lastGoalRotation)).getDegrees()) < 10.0
            ? (targetPose.getRotation().minus(lastGoalRotation)).getRadians()
                / (Timer.getTimestamp() - lastTime)
            : thetaController.getSetpoint().velocity;
    double thetaVelocity =
        thetaController.calculate(
                currentPose.getRotation().getRadians(),
                new State(targetPose.getRotation().getRadians(), thetaSetpointVelocity))
            + thetaController.getSetpoint().velocity * thetaFFScaler;
    if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;
    lastGoalRotation = targetPose.getRotation();
    lastTime = Timer.getTimestamp();

    // Command speeds
    if (drive != null) {
      drive.runVelocity(
          new ChassisVelocities(
                  driveVelocity.getX(),
                  driveVelocity.getY(),
                  omegaOverride.get().isPresent() ? omegaOverride.get().get() : thetaVelocity)
              .toRobotRelative(currentPose.getRotation()));
    }

    // Log data
    Logger.recordOutput("DriveToPose/DistanceMeasured", driveErrorAbs);
    Logger.recordOutput("DriveToPose/DistanceSetpoint", driveSetpoint.position);
    Logger.recordOutput("DriveToPose/DistanceSetpointVelocity", driveSetpoint.velocity);
    Logger.recordOutput("DriveToPose/ThetaMeasured", currentPose.getRotation().getRadians());
    Logger.recordOutput("DriveToPose/ThetaSetpoint", thetaController.getSetpoint().position);
    Logger.recordOutput(
        "DriveToPose/ThetaSetpointVelocity", thetaController.getSetpoint().velocity);
    Logger.recordOutput(
        "DriveToPose/Setpoint",
        new Pose2d[] {
          new Pose2d(
              lastSetpointTranslation,
              Rotation2d.fromRadians(thetaController.getSetpoint().position))
        });
    Logger.recordOutput("DriveToPose/Goal", new Pose2d[] {targetPose});
  }
}
