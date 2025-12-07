// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License file
// at the root directory of this project.

package org.team2342.frc.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.team2342.frc.Constants;
import org.team2342.lib.util.Timestamped;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhoton implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;
  private final PhotonPoseEstimator poseEstimator;
  private final CameraParameters parameters;

  // TODO: AllianceUtils
  /**
   * Creates a new VisionIOPhotonVision.
   *
   * @param name The configured name of the camera.
   * @param robotToCamera The 3D position of the camera relative to the robot.
   */
  public VisionIOPhoton(
      String name,
      CameraParameters parameters,
      PoseStrategy poseStrategy,
      Transform3d robotToCamera) {
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
    this.parameters = parameters;
    poseEstimator =
        new PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField),
            poseStrategy,
            robotToCamera);
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs, Timestamped<Rotation2d> heading) {
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
              Optional.ofNullable(parameters.cameraMatrix),
              Optional.ofNullable(parameters.distCoeffs),
              Constants.VisionConstants.CONSTRAINED_SOLVEPNP_PARAMETERS);
      if (optional.isEmpty()) {
        continue;
      }

      EstimatedRobotPose poseEstimate = optional.get();
      Logger.recordOutput("Vision/Camerui", poseEstimate.strategy);

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

  public record CameraParameters(Matrix<N3, N3> cameraMatrix, Matrix<N8, N1> distCoeffs) {}
}
