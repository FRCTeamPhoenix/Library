// Copyright (c) 2025 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.util;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Path;
import java.util.Arrays;
import lombok.Getter;
import lombok.Setter;

public class CameraParameters {

  @Getter @Setter private String cameraName;
  @Getter @Setter private int resWidth, resHeight;
  @Getter @Setter private Matrix<N3, N3> cameraMatrix;
  @Getter @Setter private Matrix<N8, N1> distCoeffs;
  @Getter @Setter private double avgErrorPx;
  @Getter @Setter private double errorStdDevPx;

  public CameraParameters(
      String cameraName,
      int resWidth,
      int resHeight,
      double avgErrorPx,
      double errorStdDevPx,
      Matrix<N3, N3> cameraMatrix,
      Matrix<N8, N1> distCoeffs) {
    this.cameraName = cameraName;
    this.resWidth = resWidth;
    this.resHeight = resHeight;
    this.avgErrorPx = avgErrorPx;
    this.errorStdDevPx = errorStdDevPx;
    this.cameraMatrix = cameraMatrix;
    this.distCoeffs = distCoeffs;
  }

  public CameraParameters(String cameraName, int resWidth, int resHeight) {
    this(cameraName, resWidth, resHeight, 0.02, 0.05, Rotation2d.kCCW_90deg);
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
      fovDiag = Rotation2d.fromDegrees(MathUtil.clamp(fovDiag.getDegrees(), 1, 179));
      DriverStation.reportError(
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
    this.cameraName = cameraName;
    var mapper = new ObjectMapper();
    var json = mapper.readTree(path.toFile());
    // json = json.get("calibrations");
    boolean success = false;
    try {
      for (int i = 0; i < json.size() && !success; i++) {
        // check if this calibration entry is our desired resolution
        int jsonWidth = json.get("resolution").get("width").asInt();
        int jsonHeight = json.get("resolution").get("height").asInt();
        if (jsonWidth != resWidth || jsonHeight != resHeight) continue;
        // get the relevant calibration values
        var jsonIntrinsicsNode = json.get("cameraIntrinsics").get("data");
        double[] jsonIntrinsics = new double[jsonIntrinsicsNode.size()];
        for (int j = 0; j < jsonIntrinsicsNode.size(); j++) {
          jsonIntrinsics[j] = jsonIntrinsicsNode.get(j).asDouble();
        }
        var jsonDistortNode = json.get("distCoeffs").get("data");
        double[] jsonDistortion = new double[8];
        Arrays.fill(jsonDistortion, 0);
        for (int j = 0; j < jsonDistortNode.size(); j++) {
          jsonDistortion[j] = jsonDistortNode.get(j).asDouble();
        }

        // not working
        // var jsonViewErrors = json.get("perViewErrors");
        // double jsonAvgError = 0;
        // for (int j = 0; j < jsonViewErrors.size(); j++) {
        //   jsonAvgError += jsonViewErrors.get(j).asDouble();
        // }
        // jsonAvgError /= jsonViewErrors.size();
        // double jsonErrorStdDev = json.get("standardDeviation").asDouble();

        // assign the read JSON values to this CameraProperties
        this.resWidth = jsonWidth;
        this.resHeight = jsonHeight;
        this.cameraMatrix = MatBuilder.fill(Nat.N3(), Nat.N3(), jsonIntrinsics);
        this.distCoeffs = MatBuilder.fill(Nat.N8(), Nat.N1(), jsonDistortion);
        avgErrorPx = 0.02; // jsonAvgError;
        errorStdDevPx = 0.05; // jsonErrorStdDev;
        success = true;
      }
    } catch (Exception e) {
      throw new IOException("Invalid calibration JSON");
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
      DriverStation.reportError(
          "Error while loading camera " + cameraName + ". Resorting to basic parameters", false);
      return new CameraParameters(cameraName, resWidth, resHeight);
    }
  }
}
