// Copyright (c) 2025 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.ConstrainedSolvepnpParams;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.team2342.lib.util.CameraParameters;
import org.team2342.lib.util.Timestamped;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhoton implements VisionIO {
  protected final PhotonCamera camera;
  protected final CameraParameters parameters;
  protected final Transform3d robotToCamera;
  private final PhotonPoseEstimator poseEstimator;

  private final PoseStrategy primaryStrategy;

  private boolean hasEnabled = false;

  public static final Optional<ConstrainedSolvepnpParams> CONSTRAINED_SOLVEPNP_PARAMETERS =
      Optional.of(new ConstrainedSolvepnpParams(false, 0.5));

  /**
   * Creates a new VisionIOPhotonVision.
   *
   * @param name The configured name of the camera.
   * @param robotToCamera The 3D position of the camera relative to the robot.
   */
  public VisionIOPhoton(
      CameraParameters parameters, PoseStrategy primaryStrategy, PoseStrategy disabledStrategy) {
    camera = new PhotonCamera(parameters.getCameraName());
    this.robotToCamera = parameters.getTransform();
    this.parameters = parameters;
    poseEstimator =
        new PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField),
            disabledStrategy,
            robotToCamera);
    this.primaryStrategy = primaryStrategy;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs, Timestamped<Rotation2d> heading) {
    if (!hasEnabled) {
      if (DriverStation.isEnabled()) {
        poseEstimator.setPrimaryStrategy(primaryStrategy);
        hasEnabled = true;
      }
    }

    inputs.connected = camera.isConnected();

    poseEstimator.addHeadingData(heading.getTimestamp(), heading.get());

    // Read new camera observations
    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    for (var result : camera.getAllUnreadResults()) {
      if (!result.hasTargets()) {
        continue;
      }

      Optional<EstimatedRobotPose> optional =
          poseEstimator.update(
              result,
              Optional.of(parameters.getCameraMatrix()),
              Optional.of(parameters.getDistCoeffs()),
              CONSTRAINED_SOLVEPNP_PARAMETERS);
      if (optional.isEmpty()) {
        continue;
      }

      EstimatedRobotPose poseEstimate = optional.get();

      double distance = 0;
      double ambiguity = 0;
      int tagCount = poseEstimate.targetsUsed.size();

      for (PhotonTrackedTarget target : poseEstimate.targetsUsed) {
        distance += target.getBestCameraToTarget().getTranslation().getNorm();
        ambiguity += target.poseAmbiguity;
        tagIds.add((short) target.fiducialId);
      }

      distance /= tagCount;
      ambiguity /= tagCount;

      inputs.latestTargetObservation =
          new TargetObservation(
              Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
              Rotation2d.fromDegrees(result.getBestTarget().getPitch()));

      PoseObservationType type = PoseObservationType.PHOTONVISION;
      if (poseEstimate.strategy == PoseStrategy.CONSTRAINED_SOLVEPNP) {
        type = PoseObservationType.PHOTONVISION_CONSTRAINED;
      }

      // Add observation
      poseObservations.add(
          new PoseObservation(
              result.getTimestampSeconds(), // Timestamp
              poseEstimate.estimatedPose, // 3D pose estimate
              ambiguity, // Ambiguity
              tagCount, // Tag count
              distance, // Average tag distance
              type)); // Observation type
    }

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }
}
