// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.util;

import org.wpilib.vision.apriltag.AprilTagFieldLayout;
import org.wpilib.vision.apriltag.AprilTagFields;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.driverstation.Alliance;
import org.wpilib.command2.button.Trigger;
import lombok.Getter;
import lombok.Setter;

/** Class with alliance-related utility functions */
public class AllianceUtils {

  @Getter @Setter
  private static AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

  public static void loadFieldLayout(String resourcePath) {
    try {
      fieldLayout = AprilTagFieldLayout.loadFromResource(resourcePath);
    } catch (Exception e) {
      DriverStationErrors.reportError("Failed to load AprilTagFieldLayout from resource", false);
    }
  }

  public static boolean isRedAlliance() {
    var alliance = MatchState.getAlliance();

    if (alliance.isEmpty())
      DriverStationErrors.reportError("Alliance variable is empty, defaulting to red!", false);
    return alliance.orElse(Alliance.RED) == Alliance.RED;
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
