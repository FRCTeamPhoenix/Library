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

  /** Get the angle to point at the target from a pose */
  public Rotation2d headingTo(Pose2d from, Translation2d target) {
    return target.minus(from.getTranslation()).getAngle().get();
  }

  /** Get the angle to point at the target from a pose */
  public Rotation2d headingTo(Pose2d from, Pose2d to) {
    return headingTo(from, to.getTranslation());
  }

  /** Get the distance between poses */
  public double distanceBetween(Pose2d a, Pose2d b) {
    return b.minus(a).getTranslation().getNorm();
  }

  /** Returns the inverse of a pose */
  public static Pose2d inverse(Pose2d pose) {
    Rotation2d rotationInverse = pose.getRotation().unaryMinus();
    return new Pose2d(
        pose.getTranslation().unaryMinus().rotateBy(rotationInverse), rotationInverse);
  }

  /** Create a pose with a forced translation */
  public static Pose2d withForcedTranslation(Pose2d pose, Translation2d translation) {
    return new Pose2d(translation, pose.getRotation());
  }

  /** Create a pose with a forced rotation */
  public static Pose2d withForcedRotation(Pose2d pose, Rotation2d rotation) {
    return new Pose2d(pose.getTranslation(), rotation);
  }

  /** Create a transform with a specified translation and no rotation */
  public static Transform2d toTransform2d(Translation2d translation) {
    return new Transform2d(translation, Rotation2d.ZERO);
  }

  /** Create a transform with a specified rotation and no translation */
  public static Transform2d toTransform2d(Rotation2d rotation) {
    return new Transform2d(Translation2d.ZERO, rotation);
  }

  /** Turn a Pose2d into a Transform2d */
  public static Transform2d toTransform2d(Pose2d pose) {
    return new Transform2d(pose.getTranslation(), pose.getRotation());
  }

  /** Turn a Transform2d into a Pose2d */
  public static Pose2d toPose2d(Transform2d transform) {
    return new Pose2d(transform.getTranslation(), transform.getRotation());
  }

  /** Turn a Pose3d into a Transform3d */
  public static Transform3d toTransform3d(Pose3d pose) {
    return new Transform3d(pose.getTranslation(), pose.getRotation());
  }

  /** Turn a Transform3d into a Pose3d */
  public static Pose3d toPose3d(Transform3d transform) {
    return new Pose3d(transform.getTranslation(), transform.getRotation());
  }

  /** Turn a Transform3d into a Transform2d */
  public static Transform2d toTransform2d(Transform3d transform) {
    return new Transform2d(
        transform.getTranslation().toTranslation2d(), transform.getRotation().toRotation2d());
  }

  /** Turn a Pose3d into a Pose2d */
  public static Pose2d toPose2d(Pose3d pose) {
    return new Pose2d(pose.getX(), pose.getY(), pose.getRotation().toRotation2d());
  }
}
