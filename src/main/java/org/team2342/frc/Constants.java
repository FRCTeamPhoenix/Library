// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import org.team2342.lib.util.CameraParameters;

public final class Constants {
  public static final Mode CURRENT_MODE = Mode.REAL;
  public static final boolean TUNING = true;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class VisionConstants {
    public static final String CAMERA_NAME = "left_arducam";

    public static final Transform3d CAMERA_TRANSFORM =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(7.883 + 0.5),
                Units.inchesToMeters(-10.895 - 0.5),
                Units.inchesToMeters(8)),
            new Rotation3d(0, Units.degreesToRadians(-22.0), Units.degreesToRadians(90 - 61.475)));

    public static final CameraParameters LEFT_PARAMETERS =
        CameraParameters.loadFromName(CAMERA_NAME, 800, 600).withTransform(CAMERA_TRANSFORM);

    // Basic filtering thresholds
    public static final double MAX_AMBIGUITY = 0.1;
    public static final double MAX_Z_ERROR = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static final double LINEAR_STD_DEV_BASELINE = 0.06; // Meters
    public static final double ANGULAR_STD_DEV_BASELINE = 0.12; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static final double[] CAMERA_STD_DEV_FACTORS =
        new double[] {
          1.0, // Camera 0
          1.0, // Camera 1
          1.0 // Camera 2
        };

    // Multipliers to apply for MegaTag2/ConstrainedPNP observations
    public static final double LINEAR_STD_DEV_CONSTRAINED_FACTOR =
        0.5; // More stable than full 3D solve
    public static final double ANGULAR_STD_DEV_CONSTRAINED_FACTOR =
        Double.POSITIVE_INFINITY; // No rotation data available
  }

  public static final class DriveConstants {
    public static final double CONTROLLER_DEADBAND = 0.1;
    public static final double ROTATION_LOCK_TIME = 0.25;

    public static final double MAX_LINEAR_SPEED = Units.feetToMeters(15.5);
    public static final double MAX_LINEAR_ACCELERATION = 20.0;
    public static final double DRIVE_GEARING = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    public static final double TURN_GEARING = 150.0 / 7.0;
    public static final double COUPLE_RATIO = 27.0 / 17.0 / 3;

    public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);
    public static final double WHEEL_COF = 1.2;

    public static final double TRACK_WIDTH_X = Units.inchesToMeters(28.0 - (2.625 * 2));
    public static final double TRACK_WIDTH_Y = Units.inchesToMeters(28.0 - (2.625 * 2));
    public static final double DRIVE_BASE_RADIUS =
        Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
    public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

    public static final double ROBOT_MASS_KG = Units.lbsToKilograms(112);
    public static final double ROBOT_MOI = 5.278;

    public static final double TURN_CURRENT_LIMIT = 30.0;
    public static final double SLIP_CURRENT_LIMIT = 70.0;
    public static final double DRIVE_SUPPLY_LIMIT = 40.0;
    public static final double MAX_MODULE_VELOCITY_RAD = Units.degreesToRadians(1080.0);

    public static final double[] ENCODER_OFFSETS = {
      0.229 + 0.5, 0.2834 + 0.5, 0.2009 + 0.5, 0.1563 + 0.5
    };

    // Pitch, Roll, Yaw
    public static final double[] PIGEON_CALIBRATED_MOUNT_POSE = {0, 0, 0};

    public static final boolean IS_CANFD = false;
    public static final double ODOMETRY_FREQUENCY = IS_CANFD ? 250.0 : 100.0;
  }

  public static final class CANConstants {
    public static final int PDH_ID = 14;

    public static final int PIGEON_ID = 13;
    public static final int[] FL_IDS = {1, 5, 9};
    public static final int[] FR_IDS = {2, 6, 10};
    public static final int[] BL_IDS = {3, 7, 11};
    public static final int[] BR_IDS = {4, 8, 12};
  }
}
