// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc.commands;

import org.wpilib.math.controller.PIDController;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.Trajectory;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.util.Units;
import org.wpilib.system.Timer;
import org.wpilib.command2.Command;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.team2342.frc.subsystems.drive.Drive;

public class FollowTrajectory extends Command {
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
  private Trajectory trajectory;

  private Timer timer = new Timer();

  private boolean isDone = false;

  public FollowTrajectory(Drive drive, Trajectory trajectory, Supplier<Pose2d> robotPose) {
    this.drive = drive;
    this.trajectory = trajectory;
    this.robotPose = robotPose;

    // controller.setTolerance(new Pose2d(0.01, 0.01, new Rotation2d(0.01)));

    super.addRequirements(drive);
    setName("DriveToPose");
  }

  public FollowTrajectory(Drive drive, Trajectory trajectory) {
    this(drive, trajectory, drive::getPose);
  }

  @Override
  public void initialize() {
    xController.reset();
    yController.reset();
    angleController.reset(robotPose.get().getRotation().getRadians());
    isDone = false;

    timer.restart();

    Logger.recordOutput("Odometry/Trajectory", trajectory);
    Logger.recordOutput("Trajectory/IsDone", false);
    Logger.recordOutput(
        "Odometry/TrajectorySetpoint",
        trajectory.sample(trajectory.getTotalTimeSeconds()).poseMeters);
  }

  @Override
  public void execute() {
    Pose2d currentPosition = robotPose.get();

    State nextState = trajectory.sample(timer.get());

    ChassisVelocities speeds =
        controller.calculate(currentPosition, nextState, nextState.poseMeters.getRotation());

    drive.runVelocity(speeds);
    isDone = controller.atReference();
    Logger.recordOutput("Trajectory/IsDone", isDone);
  }

  @Override
  public boolean isFinished() {
    return isDone;
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisVelocities());
  }
}
