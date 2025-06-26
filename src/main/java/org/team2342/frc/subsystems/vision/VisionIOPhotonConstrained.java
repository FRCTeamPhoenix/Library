// Copyright (c) 2025 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.cscore.OpenCvLoader;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;
import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.Logger;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.OpenCVHelp;
import org.photonvision.estimation.TargetModel;
import org.photonvision.jni.ConstrainedSolvepnpJni;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import org.team2342.frc.Constants.VisionConstants;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonConstrained implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;
  private final Supplier<Rotation2d> rotationSupplier;
  private final Matrix<N3, N3> cameraMatrix;
  private final Matrix<N8, N1> distCoeffs;

  private final TimeInterpolatableBuffer<Rotation2d> headingBuffer =
      TimeInterpolatableBuffer.createBuffer(1.0);

  private double scalingFactor = 1.0;
  private boolean headingFree = true;

  /**
   * Creates a new VisionIOPhotonVision.
   *
   * @param name The configured name of the camera.
   * @param robotToCamera The 3D position of the camera relative to the robot.
   */
  public VisionIOPhotonConstrained(
      String name,
      Transform3d robotToCamera,
      Supplier<Rotation2d> rotationSupplier,
      Matrix<N3, N3> cameraMatrix,
      Matrix<N8, N1> distCoeffs) {
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
    this.rotationSupplier = rotationSupplier;
    this.cameraMatrix = cameraMatrix;
    this.distCoeffs = distCoeffs;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    Rotation2d gyro = rotationSupplier.get();
    if (gyro != null) headingBuffer.addSample(Timer.getFPGATimestamp(), gyro);

    // Read new camera observations
    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    for (var result : camera.getAllUnreadResults()) {
      // Update latest target observation
      if (result.hasTargets()) {
        inputs.latestTargetObservation =
            new TargetObservation(
                Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
      } else {
        inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
      }

      if (!result.targets.isEmpty()) {
        PoseObservation observation = constrainedPNP(result, tagIds);
        if (observation != null) poseObservations.add(observation);
      }
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

  private PoseObservation multitag(PhotonPipelineResult result, Set<Short> tagIds) {
    if (!result.multitagResult.isPresent()
        && !result.targets.isEmpty()) { // Check for Multitag result
      return basic(result, tagIds); // Fallback
    }
    var multitagResult = result.multitagResult.get();

    // Calculate robot pose
    Transform3d fieldToCamera = multitagResult.estimatedPose.best;
    Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
    Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

    // Calculate average tag distance
    double totalTagDistance = 0.0;
    for (var target : result.targets) {
      totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
    }

    // Add tag IDs
    tagIds.addAll(multitagResult.fiducialIDsUsed);

    // Add observation
    return new PoseObservation(
        result.getTimestampSeconds(), // Timestamp
        robotPose, // 3D pose estimate
        multitagResult.estimatedPose.ambiguity, // Ambiguity
        multitagResult.fiducialIDsUsed.size(), // Tag count
        totalTagDistance / result.targets.size(), // Average tag distance
        PoseObservationType.PHOTONVISION); // Observation type
  }

  private PoseObservation basic(PhotonPipelineResult result, Set<Short> tagIds) {
    var target = result.targets.get(0);

    // Calculate robot pose
    var tagPose = VisionConstants.TAG_LAYOUT.getTagPose(target.fiducialId);
    if (tagPose.isPresent()) {
      Transform3d fieldToTarget =
          new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
      Transform3d cameraToTarget = target.bestCameraToTarget;
      Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
      Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
      Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

      // Add tag ID
      tagIds.add((short) target.fiducialId);

      // Add observation
      return new PoseObservation(
          result.getTimestampSeconds(), // Timestamp
          robotPose, // 3D pose estimate
          target.poseAmbiguity, // Ambiguity
          1, // Tag count
          cameraToTarget.getTranslation().getNorm(), // Average tag distance
          PoseObservationType.PHOTONVISION); // Observation type
    } else {
      // It's so over, just give up
      return null;
    }
  }

  private PoseObservation constrainedPNP(PhotonPipelineResult result, Set<Short> tagIds) {
    // Need calibrations
    if (distCoeffs == null || cameraMatrix == null) {
      return multitag(result, tagIds);
    }

    // Need heading
    if (headingBuffer.getSample(result.getTimestampSeconds()).isEmpty()) {
      return multitag(result, tagIds);
    }

    Pose3d fieldToRobotSeed;
    var multitagResult = result.getMultiTagResult();

    double tagDist;
    int tagCount;
    PoseObservationType type;

    // Attempt to use multi-tag to get a pose estimate seed
    if (multitagResult.isPresent()) {
      fieldToRobotSeed =
          Pose3d.kZero.plus(multitagResult.get().estimatedPose.best.plus(robotToCamera.inverse()));
      tagIds.addAll(multitagResult.get().fiducialIDsUsed);

      tagCount = multitagResult.get().fiducialIDsUsed.size(); // Tag count
      double totalTagDistance = 0.0;
      for (var target : result.targets) {
        totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
      }
      tagDist = totalTagDistance / result.targets.size(); // Average tag distance
    } else {
      var target = result.targets.get(0);
      var tagPose = VisionConstants.TAG_LAYOUT.getTagPose(target.fiducialId);
      if (tagPose.isPresent()) {
        Transform3d fieldToTarget =
            new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
        Transform3d cameraToTarget = target.bestCameraToTarget;
        Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
        Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());

        tagCount = 1;
        tagDist = cameraToTarget.getTranslation().getNorm(); // Average tag distance
        fieldToRobotSeed = Pose3d.kZero.plus(fieldToRobot);
      } else {
        // It's so over, just give up
        return null;
      }
      tagIds.add((short) target.fiducialId);
    }

    if (!headingFree) {
      fieldToRobotSeed =
          new Pose3d(
              fieldToRobotSeed.getTranslation(),
              new Rotation3d(headingBuffer.getSample(result.getTimestampSeconds()).get()));
      type = PoseObservationType.PHOTONVISION_CONSTRAINED;
    } else {
      type = PoseObservationType.PHOTONVISION;
    }

    Logger.recordOutput("Vision/Constrained/Seed", fieldToRobotSeed);

    var constrainedResult =
        estimatePoseConstrained(
            cameraMatrix,
            distCoeffs,
            result.getTargets(),
            robotToCamera,
            fieldToRobotSeed,
            VisionConstants.TAG_LAYOUT,
            TargetModel.kAprilTag36h11,
            headingFree,
            headingBuffer.getSample(result.getTimestampSeconds()),
            scalingFactor);

    // go to fallback if solvePNP fails for some reason
    if (!constrainedResult.isPresent()) {
      return multitag(result, tagIds);
    }

    return new PoseObservation(
        result.getTimestampSeconds(), // Timestamp
        constrainedResult.get(), // 3D pose estimate
        0.0, // Ambiguity
        tagCount, // Tag count
        tagDist, // Average tag distance
        type); // Observation type
  }

  @Override
  public void toggleHeadingFree() {
    headingFree = !headingFree;
  }

  /**
   * "Borrowed" from PhotonVision
   *
   * <p>Performs constrained solvePNP using 3d-2d point correspondences of visible AprilTags to
   * estimate the field-to-camera transformation. *
   *
   * @param cameraMatrix The camera intrinsics matrix in standard opencv form
   * @param distCoeffs The camera distortion matrix in standard opencv form
   * @param visTags The visible tags reported by PV. Non-tag targets are automatically excluded.
   * @param robot2camera The {@link Transform3d} from the robot odometry frame to the camera optical
   *     frame
   * @param robotPoseSeed An initial guess at robot pose, refined via optimizaiton. Better guesses
   *     will converge faster.
   * @param tagLayout The known tag layout on the field
   * @param tagModel The physical size of the AprilTags
   * @param headingFree If heading is completely free, or if our measured gyro is taken into account
   * @param gyro If headingFree is false, the best estimate at robot yaw. Excursions from this are
   *     penalized in our cost function.
   * @param gyroErrorScaleFac A relative weight for gyro heading excursions against tag corner
   *     reprojection error.
   * @return The camera pose. Ensure the {@link Pose3d} is present before utilizing it.
   */
  public static Optional<Pose3d> estimatePoseConstrained(
      Matrix<N3, N3> cameraMatrix,
      Matrix<N8, N1> distCoeffs,
      List<PhotonTrackedTarget> visTags,
      Transform3d robot2camera,
      Pose3d robotPoseSeed,
      AprilTagFieldLayout tagLayout,
      TargetModel tagModel,
      boolean headingFree,
      Optional<Rotation2d> gyro,
      double gyroErrorScaleFac) {
    if (tagLayout == null
        || visTags == null
        || tagLayout.getTags().isEmpty()
        || visTags.isEmpty()) {
      return Optional.empty();
    }

    var corners = new ArrayList<TargetCorner>();
    var knownTags = new ArrayList<AprilTag>();
    // ensure these are AprilTags in our layout
    for (var tgt : visTags) {
      int id = tgt.getFiducialId();
      tagLayout
          .getTagPose(id)
          .ifPresent(
              pose -> {
                knownTags.add(new AprilTag(id, pose));
                corners.addAll(tgt.getDetectedCorners());
              });
    }
    if (knownTags.isEmpty() || corners.isEmpty() || corners.size() % 4 != 0) {
      return Optional.empty();
    }
    OpenCvLoader.forceStaticLoad();

    Point[] points = OpenCVHelp.cornersToPoints(corners);

    // Undistort
    {
      MatOfPoint2f temp = new MatOfPoint2f();
      MatOfDouble cameraMatrixMat = new MatOfDouble();
      MatOfDouble distCoeffsMat = new MatOfDouble();
      OpenCVHelp.matrixToMat(cameraMatrix.getStorage()).assignTo(cameraMatrixMat);
      OpenCVHelp.matrixToMat(distCoeffs.getStorage()).assignTo(distCoeffsMat);

      temp.fromArray(points);
      Calib3d.undistortImagePoints(temp, temp, cameraMatrixMat, distCoeffsMat);
      points = temp.toArray();

      temp.release();
      cameraMatrixMat.release();
      distCoeffsMat.release();
    }

    // Rotate from wpilib to opencv camera CS
    var robot2cameraBase =
        MatBuilder.fill(Nat.N4(), Nat.N4(), 0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1);
    var robotToCamera = robot2camera.toMatrix().times(robot2cameraBase);

    // Where we saw corners
    var point_observations = new SimpleMatrix(2, points.length);
    for (int i = 0; i < points.length; i++) {
      point_observations.set(0, i, points[i].x);
      point_observations.set(1, i, points[i].y);
    }

    // Affine corner locations
    var objectTrls = new ArrayList<Translation3d>();
    for (var tag : knownTags) objectTrls.addAll(tagModel.getFieldVertices(tag.pose));
    var field2points = new SimpleMatrix(4, points.length);
    for (int i = 0; i < objectTrls.size(); i++) {
      field2points.set(0, i, objectTrls.get(i).getX());
      field2points.set(1, i, objectTrls.get(i).getY());
      field2points.set(2, i, objectTrls.get(i).getZ());
      field2points.set(3, i, 1);
    }

    // fx fy cx cy
    double[] cameraCal =
        new double[] {
          cameraMatrix.get(0, 0),
          cameraMatrix.get(1, 1),
          cameraMatrix.get(0, 2),
          cameraMatrix.get(1, 2),
        };

    var guess2 = robotPoseSeed.toPose2d();

    var ret =
        ConstrainedSolvepnpJni.do_optimization(
            headingFree,
            knownTags.size(),
            cameraCal,
            robotToCamera.getData(),
            new double[] {
              guess2.getX(), guess2.getY(), guess2.getRotation().getRadians(),
            },
            field2points.getDDRM().getData(),
            point_observations.getDDRM().getData(),
            gyro.orElse(Rotation2d.kZero).getRadians(),
            gyroErrorScaleFac);

    if (ret == null) {
      return Optional.empty();
    } else {
      Pose3d poseResult = new Pose3d(new Pose2d(ret[0], ret[1], new Rotation2d(ret[2])));
      return Optional.of(poseResult);
    }
  }
}
