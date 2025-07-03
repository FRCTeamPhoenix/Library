// Copyright (c) 2025 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.sensors.absolute;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface AbsoluteEncoderIO {
  @AutoLog
  public class AbsoluteEncoderIOInputs {
    public boolean connected = false;
    public Rotation2d angle = new Rotation2d();
  }

  public default void updateInputs(AbsoluteEncoderIOInputs inputs) {}

  public default double getNoOffsetAngle() {
    return 0.0;
  }

  public record AbsoluteEncoderConfig(double offsetRot, boolean invert) {}
}
