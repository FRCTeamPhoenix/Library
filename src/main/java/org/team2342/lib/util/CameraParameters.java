// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.util;

import io.avaje.json.JsonIoException;
import io.avaje.jsonb.Json;
import io.avaje.jsonb.Jsonb;
import java.io.FileInputStream;
import java.io.IOException;
import java.nio.file.Path;
import java.util.Arrays;
import lombok.Getter;
import lombok.Setter;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.linalg.MatBuilder;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.numbers.N8;
import org.wpilib.math.util.Nat;
import org.wpilib.system.Filesystem;

public class CameraParameters {
  @Json
  record SimCameraData(SimCameraCalibrationData[] calibrations) {
    @Json
    record SimCameraCalibrationData(
        ResolutionData resolution,
        CameraIntrinsicsData cameraIntrinsics,
        DistortionCoefficients distCoeffs,
        double[] perViewErrors,
        double standardDeviation) {
      @Json
      record ResolutionData(int width, int height) {}

      @Json
      record CameraIntrinsicsData(double[] data) {}

      @Json
      record DistortionCoefficients(double[] data) {}
    }
  }

  @Getter @Setter private String cameraName;
  @Getter @Setter private int resWidth, resHeight;
  @Getter @Setter private Matrix<N3, N3> cameraMatrix;
  @Getter @Setter private Matrix<N8, N1> distCoeffs;
  @Getter @Setter private double avgErrorPx;
  @Getter @Setter private double errorStdDevPx;
  @Getter @Setter private Transform3d transform;

  public CameraParameters(
      String cameraName,
      int resWidth,
      int resHeight,
      double avgErrorPx,
      double errorStdDevPx,
      Matrix<N3, N3> cameraMatrix,
      Matrix<N8, N1> distCoeffs,
      Transform3d transform) {
    this.cameraName = cameraName;
    this.resWidth = resWidth;
    this.resHeight = resHeight;
    this.avgErrorPx = avgErrorPx;
    this.errorStdDevPx = errorStdDevPx;
    this.cameraMatrix = cameraMatrix;
    this.distCoeffs = distCoeffs;
    this.transform = transform;
  }

  public CameraParameters(String cameraName, int resWidth, int resHeight) {
    this(cameraName, resWidth, resHeight, 0.02, 0.05, Rotation2d.CCW_90DEG);
  }

  public CameraParameters(
      String cameraName,
      int resWidth,
      int resHeight,
      double avgErrorPx,
      double errorStdDevPx,
      Rotation2d fovDiag) {
    this.cameraName = cameraName;
    this.resWidth = resWidth;
    this.resHeight = resHeight;
    this.avgErrorPx = avgErrorPx;
    this.errorStdDevPx = errorStdDevPx;

    if (fovDiag.getDegrees() < 1 || fovDiag.getDegrees() > 179) {
      fovDiag = Rotation2d.fromDegrees(Math.clamp(fovDiag.getDegrees(), 1, 179));
      DriverStationErrors.reportError(
          "Requested invalid FOV! Clamping between (1, 179) degrees...", false);
    }
    double resDiag = Math.hypot(resWidth, resHeight);
    double diagRatio = Math.tan(fovDiag.getRadians() / 2);
    var fovWidth = new Rotation2d(Math.atan(diagRatio * (resWidth / resDiag)) * 2);
    var fovHeight = new Rotation2d(Math.atan(diagRatio * (resHeight / resDiag)) * 2);

    // assume no distortion
    distCoeffs = VecBuilder.fill(0, 0, 0, 0, 0, 0, 0, 0);

    // assume centered principal point (pixels)
    double cx = resWidth / 2.0 - 0.5;
    double cy = resHeight / 2.0 - 0.5;

    // use given fov to determine focal point (pixels)
    double fx = cx / Math.tan(fovWidth.getRadians() / 2.0);
    double fy = cy / Math.tan(fovHeight.getRadians() / 2.0);

    // create camera intrinsics matrix
    cameraMatrix = MatBuilder.fill(Nat.N3(), Nat.N3(), fx, 0, cx, 0, fy, cy, 0, 0, 1);
  }

  public CameraParameters(String cameraName, int resWidth, int resHeight, Path path)
      throws IOException {
    SimCameraData data;
    this.cameraName = cameraName;
    try (var stream = new FileInputStream(path.toFile())) {
      data = Jsonb.instance().type(SimCameraData.class).fromJson(stream);
    } catch (JsonIoException e) {
      throw new IOException("Invalid calibration JSON", e);
    }
    boolean success = false;
    for (var calib : data.calibrations) {
      // check if this calibration entry is our desired resolution
      if (calib.resolution.width != resWidth || calib.resolution.height != resHeight) continue;
      // get the relevant calibration values
      double avgViewError = Arrays.stream(calib.perViewErrors).average().orElse(0);
      // assign the read JSON values to this CameraProperties
      resWidth = calib.resolution.width;
      resHeight = calib.resolution.height;
      cameraMatrix = MatBuilder.fill(Nat.N3(), Nat.N3(), calib.cameraIntrinsics.data);
      distCoeffs = MatBuilder.fill(Nat.N8(), Nat.N1(), calib.distCoeffs.data);
      avgErrorPx = avgViewError;
      errorStdDevPx = calib.standardDeviation;
    }
    if (!success) throw new IOException("Requested resolution not found in calibration");
  }

  public static CameraParameters loadFromName(String cameraName, int resWidth, int resHeight) {
    try {
      return new CameraParameters(
          cameraName,
          resWidth,
          resHeight,
          Filesystem.getDeployDirectory()
              .toPath()
              .resolve("calibrations/" + cameraName + "_" + resWidth + ".json"));
    } catch (Exception e) {
      System.out.println(e);
      DriverStationErrors.reportError(
          "Error while loading camera " + cameraName + ". Resorting to basic parameters", false);
      return new CameraParameters(cameraName, resWidth, resHeight);
    }
  }

  public CameraParameters withTransform(Transform3d transform) {
    this.transform = transform;
    return this;
  }
}
