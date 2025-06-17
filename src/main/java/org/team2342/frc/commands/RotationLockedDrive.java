package org.team2342.frc.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.team2342.frc.Constants.DriveConstants;
import org.team2342.frc.subsystems.drive.Drive;

public class RotationLockedDrive extends Command {

  private final DoubleSupplier xSupplier, ySupplier, rotationSupplier;
  private final Drive drive;

  private final Timer inputTimer = new Timer();
  private final ProfiledPIDController rotationPID =
      new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(8.0, 20.0));
  private Rotation2d rotationLock;

  public RotationLockedDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotationSupplier) {
    this.drive = drive;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.rotationSupplier = rotationSupplier;

    rotationPID.enableContinuousInput(-Math.PI, Math.PI);

    super.setName("RotationLockedDrive");
    super.addRequirements(drive);
  }

  @Override
  public void initialize() {
    inputTimer.restart();
    rotationLock = drive.getRawOdometryPose().getRotation();
    rotationPID.reset(drive.getRotation().getRadians());
  }

  @Override
  public void execute() {
    // Calculate rotational velocity
    double controllerOmega =
        MathUtil.applyDeadband(rotationSupplier.getAsDouble(), DriveConstants.CONTROLLER_DEADBAND);
    controllerOmega = Math.copySign(controllerOmega * controllerOmega, controllerOmega);

    if (controllerOmega != 0) {
      inputTimer.reset();
    }

    double omega;
    if (inputTimer.hasElapsed(DriveConstants.ROTATION_LOCK_TIME)) {
      Logger.recordOutput("Drive/RotationLock/Engaged", true);
      Logger.recordOutput("Drive/RotationLock/LockedHeading", rotationLock);
      omega =
          rotationPID.calculate(
              drive.getRawOdometryPose().getRotation().getRadians(), rotationLock.getRadians());
    } else {
      Logger.recordOutput("Drive/RotationLock/Engaged", false);
      omega = controllerOmega * drive.getMaxAngularSpeedRadPerSec();
      rotationLock = drive.getRawOdometryPose().getRotation();
      rotationPID.reset(drive.getRotation().getRadians());
    }

    // Calculate new linear velocity
    Translation2d linearVelocity =
        DriveCommands.getLinearVelocityFromJoysticks(
            xSupplier.getAsDouble(), ySupplier.getAsDouble());

    // Convert to field relative speeds & send command
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
            omega,
            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    Logger.recordOutput("Drive/RotationLock/Engaged", false);
    inputTimer.stop();
  }
}
