// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.Getter;
import lombok.Setter;

/** Class with alliance-related utility functions */
public class AllianceUtils {

  @Getter @Setter
  private static AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

  public static boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();

    if (alliance.isEmpty())
      DriverStation.reportError("Alliance variable is empty, defaulting to red!", false);
    return alliance.orElse(Alliance.Red) == Alliance.Red;
  }

  public static boolean isBlueAlliance() {
    return !isRedAlliance();
  }

  public static Trigger driverStationAttachedTrigger() {
    return new Trigger(DriverStation::isDSAttached);
  }

  public static Pose2d flipToAlliance(Pose2d bluePose, AprilTagFieldLayout field) {
    return isRedAlliance()
        ? new Pose2d(
            field.getFieldLength() - bluePose.getX(),
            field.getFieldWidth() - bluePose.getY(),
            bluePose.getRotation().rotateBy(Rotation2d.kPi))
        : bluePose;
  }

  public static Pose3d flipToAlliance(Pose3d bluePose, AprilTagFieldLayout field) {
    return isRedAlliance()
        ? new Pose3d(
            field.getFieldLength() - bluePose.getX(),
            field.getFieldWidth() - bluePose.getY(),
            bluePose.getZ(),
            bluePose.getRotation().rotateBy(new Rotation3d(0.0, 0.0, Math.PI)))
        : bluePose;
  }

  public static Translation2d flipToAlliance(
      Translation2d blueTranslation, AprilTagFieldLayout field) {
    return isRedAlliance()
        ? new Translation2d(
            field.getFieldLength() - blueTranslation.getX(),
            field.getFieldWidth() - blueTranslation.getY())
        : blueTranslation;
  }

  public static Translation3d flipToAlliance(
      Translation3d blueTranslation, AprilTagFieldLayout field) {
    return isRedAlliance()
        ? new Translation3d(
            field.getFieldLength() - blueTranslation.getX(),
            field.getFieldWidth() - blueTranslation.getY(),
            blueTranslation.getZ())
        : blueTranslation;
  }

  public static Pose2d flipToAlliance(Pose2d bluePose) {
    return flipToAlliance(bluePose, fieldLayout);
  }

  public static Pose3d flipToAlliance(Pose3d bluePose) {
    return flipToAlliance(bluePose, fieldLayout);
  }

  public static Translation2d flipToAlliance(Translation2d blueTranslation) {
    return flipToAlliance(blueTranslation, fieldLayout);
  }

  public static Translation3d flipToAlliance(Translation3d blueTranslation) {
    return flipToAlliance(blueTranslation, fieldLayout);
  }
}
