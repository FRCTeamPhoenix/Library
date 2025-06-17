package org.team2342.frc;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import lombok.Getter;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.team2342.frc.Constants.CANConstants;
import org.team2342.frc.Constants.DriveConstants;
import org.team2342.frc.Constants.VisionConstants;
import org.team2342.frc.commands.DriveCommands;
import org.team2342.frc.commands.DriveToPose;
import org.team2342.frc.commands.RotationLockedDrive;
import org.team2342.frc.subsystems.drive.Drive;
import org.team2342.frc.subsystems.drive.GyroIO;
import org.team2342.frc.subsystems.drive.GyroIOPigeon2;
import org.team2342.frc.subsystems.drive.ModuleIO;
import org.team2342.frc.subsystems.drive.ModuleIOSim;
import org.team2342.frc.subsystems.drive.ModuleIOTalonFX;
import org.team2342.frc.subsystems.vision.Vision;
import org.team2342.frc.subsystems.vision.VisionIO;
import org.team2342.frc.subsystems.vision.VisionIOConstrainedSim;
import org.team2342.frc.subsystems.vision.VisionIOPhoton;
import org.team2342.frc.subsystems.vision.VisionIOSim;

public class RobotContainer {
  @Getter private final Drive drive;
  @Getter private final Vision vision;

  private final LoggedDashboardChooser<Command> autoChooser;

  @Getter private final CommandXboxController driverController = new CommandXboxController(0);
  private final Alert driverControllerAlert =
      new Alert("Driver controller is disconnected!", AlertType.kError);

  public RobotContainer() {
    switch (Constants.CURRENT_MODE) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(CANConstants.PIGEON_ID),
                new ModuleIOTalonFX(CANConstants.FL_IDS, DriveConstants.COMP_ENCODER_OFFSETS[0]),
                new ModuleIOTalonFX(CANConstants.FR_IDS, DriveConstants.COMP_ENCODER_OFFSETS[1]),
                new ModuleIOTalonFX(CANConstants.BL_IDS, DriveConstants.COMP_ENCODER_OFFSETS[2]),
                new ModuleIOTalonFX(CANConstants.BR_IDS, DriveConstants.COMP_ENCODER_OFFSETS[3]));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhoton(
                    VisionConstants.RIGHT_CAMERA_NAME, VisionConstants.FRONT_RIGHT_TRANSFORM),
                new VisionIOPhoton(
                    VisionConstants.LEFT_CAMERA_NAME, VisionConstants.FRONT_LEFT_TRANSFORM));

        LoggedPowerDistribution.getInstance(CANConstants.PDH_ID, ModuleType.kRev);
        break;

      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOConstrainedSim(
                    VisionConstants.RIGHT_CAMERA_NAME,
                    VisionConstants.FRONT_RIGHT_TRANSFORM,
                    drive::getVisionGyroHeading,
                    drive::getRawOdometryPose,
                    VisionConstants.cameraMatrix,
                    VisionConstants.distCoeffs),
                new VisionIOSim(
                    VisionConstants.LEFT_CAMERA_NAME,
                    VisionConstants.FRONT_LEFT_TRANSFORM,
                    drive::getRawOdometryPose));

        break;

      default:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        break;
    }

    configureNamedCommands();

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser.get();

    setupDevelopmentRoutines();

    configureBindings();

    SmartDashboard.putData(
        "Print Encoder Zeros",
        Commands.runOnce(() -> drive.printModuleAbsoluteAngles()).ignoringDisable(true));

    SmartDashboard.putData(
        "Set Vision Gyro Offset",
        Commands.runOnce(() -> drive.setVisionGyroOffset()).ignoringDisable(true));
    SmartDashboard.putData(
        "Toggle Constrained PhotonVision",
        Commands.runOnce(() -> vision.toggleHeadingsFree()).ignoringDisable(true));
  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand("Named Command Test", Commands.print("Named Command Test"));
  }

  private void configureBindings() {
    // Basic drive controls
    drive.setDefaultCommand(
        new RotationLockedDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    driverController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
    driverController
        .a()
        .whileTrue(
            new DriveToPose(
                drive,
                VisionConstants.TAG_LAYOUT
                    .getTagPose(7)
                    .orElse(new Pose3d())
                    .toPose2d()
                    .plus(
                        new Transform2d(
                            DriveConstants.DRIVE_BASE_RADIUS + 0.45, 0, Rotation2d.k180deg)),
                drive::getPose,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX()));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  private void setupDevelopmentRoutines() {
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  public void updateAlerts() {
    driverControllerAlert.set(!driverController.isConnected());
  }
}
