package org.team2342.frc.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.Queue;
import org.team2342.frc.Constants.DriveConstants;
import org.team2342.frc.util.PhoenixUtils;

/** GyroIO implementation for the Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon;
  private final StatusSignal<Angle> yaw;
  private final StatusSignal<AngularVelocity> yawVelocity;

  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  // Connection debouncer
  private final Debouncer gyroConnectedDebounce = new Debouncer(0.5);

  public GyroIOPigeon2(int canID) {
    pigeon = new Pigeon2(canID);
    yaw = pigeon.getYaw();
    yawVelocity = pigeon.getAngularVelocityZWorld();

    // Get the MountPose configuration from Tuner X
    Pigeon2Configuration config =
        new Pigeon2Configuration()
            .withMountPose(
                new MountPoseConfigs()
                    .withMountPosePitch(DriveConstants.PIGEON_CALIBRATED_MOUNT_POSE[0])
                    .withMountPoseRoll(DriveConstants.PIGEON_CALIBRATED_MOUNT_POSE[1])
                    .withMountPoseYaw(DriveConstants.PIGEON_CALIBRATED_MOUNT_POSE[2]));

    PhoenixUtils.tryUntilOk(5, () -> pigeon.getConfigurator().apply(config));
    PhoenixUtils.tryUntilOk(5, () -> pigeon.getConfigurator().setYaw(0.0));

    // We want the yaw signal to run at a higher frequency for odometry
    yaw.setUpdateFrequency(DriveConstants.ODOMETRY_FREQUENCY);
    yawTimestampQueue = PhoenixOdometry.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometry.getInstance().registerSignal(yaw.clone());

    yawVelocity.setUpdateFrequency(50.0);

    // Optimize utilization for Pigeon 2
    PhoenixUtils.tryUntilOk(5, () -> pigeon.optimizeBusUtilization());

    // Register signals for updating
    PhoenixUtils.registerSignals(yaw, yawVelocity);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected =
        gyroConnectedDebounce.calculate(BaseStatusSignal.isAllGood(yaw, yawVelocity));
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
