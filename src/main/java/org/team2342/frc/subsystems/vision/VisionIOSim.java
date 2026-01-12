// Copyright (c) 2025 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.Supplier;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.team2342.lib.util.AllianceUtils;
import org.team2342.lib.util.CameraParameters;
import org.team2342.lib.util.Timestamped;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOSim extends VisionIOPhoton {
  private static VisionSystemSim visionSim;

  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;

  /**
   * Creates a new VisionIOPhotonVisionSim.
   *
   * @param name The name of the camera.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   */
  public VisionIOSim(
      CameraParameters parameters,
      PoseStrategy primaryStrategy,
      PoseStrategy disabledStrategy,
      Supplier<Pose2d> poseSupplier) {
    super(parameters, primaryStrategy, disabledStrategy);
    this.poseSupplier = poseSupplier;

    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(AllianceUtils.getFieldLayout());
    }

    // Add sim camera
    SimCameraProperties properties = new SimCameraProperties();
    properties.setCalibration(
        parameters.getResWidth(),
        parameters.getResHeight(),
        parameters.getCameraMatrix(),
        parameters.getDistCoeffs());
    properties.setCalibError(parameters.getAvgErrorPx(), parameters.getErrorStdDevPx());
    properties.setFPS(60.0);
    properties.setAvgLatencyMs(35);
    properties.setLatencyStdDevMs(7);

    cameraSim = new PhotonCameraSim(camera, properties, AllianceUtils.getFieldLayout());
    visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs, Timestamped<Rotation2d> heading) {
    visionSim.update(poseSupplier.get());
    super.updateInputs(inputs, heading);
  }
}
