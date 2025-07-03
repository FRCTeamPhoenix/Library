// Copyright (c) 2025 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.sensors.absolute;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.DoubleSupplier;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class AbsoluteEncoderIOSim implements AbsoluteEncoderIO {

  private final DoubleSupplier angleRadSupplier;

  @Override
  public void updateInputs(AbsoluteEncoderIOInputs inputs) {
    inputs.connected = true;
    inputs.angle = Rotation2d.fromRadians(angleRadSupplier.getAsDouble());
  }
}
