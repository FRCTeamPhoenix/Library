// Copyright (c) 2025 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.team2342.frc.subsystems.drive.Drive;

public class DriveToPose extends Command {
  private PIDController xController = new PIDController(4.0, 0.0, 0.02);
  private PIDController yController = new PIDController(4.0, 0.0, 0.02);
  private ProfiledPIDController angleController =
      new ProfiledPIDController(
          2.5,
          0.0,
          0.02,
          new TrapezoidProfile.Constraints(
              Units.degreesToRadians(1080), Units.degreesToRadians(1260)));

  private HolonomicDriveController controller =
      new HolonomicDriveController(xController, yController, angleController);

  private final Drive drive;
  private final Supplier<Pose2d> robotPose;
  private Pose2d target;

  private DoubleSupplier xSupplier;
  private DoubleSupplier ySupplier;

  private boolean isDone = false;
  private double multiplier = 1.2;

  public DriveToPose(
      Drive drive,
      Pose2d targetPose,
      Supplier<Pose2d> robotPose,
      DoubleSupplier xNudge,
      DoubleSupplier yNudge) {
    this.drive = drive;
    this.target = targetPose;
    this.robotPose = robotPose;
    this.xSupplier = xNudge;
    this.ySupplier = yNudge;

    // controller.setTolerance(new Pose2d(0.01, 0.01, new Rotation2d(0.01)));

    super.addRequirements(drive);
    setName("DriveToPose");
  }

  public DriveToPose(Drive drive, Pose2d targetPose, Supplier<Pose2d> robotPose) {
    this(drive, targetPose, robotPose, () -> 0.0, () -> 0.0);
  }

  public DriveToPose(Drive drive, Pose2d targetPose) {
    this(drive, targetPose, drive::getPose, () -> 0.0, () -> 0.0);
  }

  @Override
  public void initialize() {
    xController.reset();
    yController.reset();
    angleController.reset(robotPose.get().getRotation().getRadians());
    isDone = false;
    Logger.recordOutput("PoseAlignment/AtGoal", false);
    Logger.recordOutput("PoseAlignment/Target", target);
  }

  @Override
  public void execute() {
    Pose2d currentPosition = robotPose.get();

    var offset = currentPosition.relativeTo(target);
    double yDistance = Math.abs(offset.getY());
    double xDistance = Math.abs(offset.getX());
    double shiftX = MathUtil.clamp((yDistance / 1.75 + ((xDistance - 0.25) / 2.75)), 0.0, 1.0);
    double shiftY = MathUtil.clamp(offset.getX() / 0.9, 0.0, 1.0);
    Pose2d goal =
        target.plus(
            new Transform2d(
                -shiftX * multiplier,
                Math.copySign(shiftY * multiplier, offset.getY()) * 0.75,
                new Rotation2d()));
    Logger.recordOutput("PoseAlignment/ShiftedTarget", goal);

    ChassisSpeeds speeds = controller.calculate(currentPosition, goal, 0, target.getRotation());

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    Translation2d joystickTranslation =
        DriveCommands.getLinearVelocityFromJoysticks(
            xSupplier.getAsDouble(), ySupplier.getAsDouble());
    double controllerBias = MathUtil.clamp(joystickTranslation.getNorm(), 0.0, 1.0);
    ChassisSpeeds controlled =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            joystickTranslation.getX() * drive.getMaxLinearSpeedMetersPerSec(),
            joystickTranslation.getY() * drive.getMaxLinearSpeedMetersPerSec(),
            0.0,
            isFlipped
                ? currentPosition.getRotation().plus(new Rotation2d(Math.PI))
                : currentPosition.getRotation());

    speeds.vxMetersPerSecond =
        MathUtil.interpolate(
            speeds.vxMetersPerSecond, controlled.vxMetersPerSecond, controllerBias);
    speeds.vyMetersPerSecond =
        MathUtil.interpolate(
            speeds.vyMetersPerSecond, controlled.vyMetersPerSecond, controllerBias);

    drive.runVelocity(speeds);
    isDone = controller.atReference();
    Logger.recordOutput("PoseAlignment/AtGoal", isDone);
  }

  @Override
  public boolean isFinished() {
    return isDone;
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds());
  }

  public void setNewTarget(Pose2d target) {
    this.target = target;

    isDone = false;
  }
}
