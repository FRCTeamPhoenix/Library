// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.util;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;

public class GeometryUtils {

  public Rotation2d headingTo(Pose2d from, Translation2d target) {
    return target.minus(from.getTranslation()).getAngle().get();
  }

  public Rotation2d headingTo(Pose2d from, Pose2d to) {
    return headingTo(from, to.getTranslation());
  }

  public double distanceBetween(Pose2d a, Pose2d b) {
    return b.minus(a).getTranslation().getNorm();
  }

  public static Pose2d inverse(Pose2d pose) {
    Rotation2d rotationInverse = pose.getRotation().unaryMinus();
    return new Pose2d(
        pose.getTranslation().unaryMinus().rotateBy(rotationInverse), rotationInverse);
  }

  public static Pose2d withTranslation(Pose2d pose, Translation2d translation) {
    return new Pose2d(translation, pose.getRotation());
  }

  public static Pose2d withRotation(Pose2d pose, Rotation2d rotation) {
    return new Pose2d(pose.getTranslation(), rotation);
  }

  public static Transform2d toTransform2d(Translation2d translation) {
    return new Transform2d(translation, Rotation2d.ZERO);
  }

  public static Transform2d toTransform2d(Rotation2d rotation) {
    return new Transform2d(Translation2d.ZERO, rotation);
  }

  public static Transform2d toTransform2d(Pose2d pose) {
    return new Transform2d(pose.getTranslation(), pose.getRotation());
  }

  public static Pose2d toPose2d(Transform2d transform) {
    return new Pose2d(transform.getTranslation(), transform.getRotation());
  }

  public static Pose3d toPose3d(Transform3d transform) {
    return new Pose3d(transform.getTranslation(), transform.getRotation());
  }

  public static Pose2d toPose2d(Translation2d translation) {
    return new Pose2d(translation, Rotation2d.ZERO);
  }

  public static Pose2d toPose2d(Rotation2d rotation) {
    return new Pose2d(Translation2d.ZERO, rotation);
  }

  public static Transform2d toTransform2d(Transform3d transform) {
    return new Transform2d(
        transform.getTranslation().toTranslation2d(), transform.getRotation().toRotation2d());
  }

  public static Transform3d toTransform3d(Pose3d pose) {
    return new Transform3d(pose.getTranslation(), pose.getRotation());
  }
}
