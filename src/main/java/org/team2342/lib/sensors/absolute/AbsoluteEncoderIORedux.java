// Copyright (c) 2025 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.sensors.absolute;

import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.reduxrobotics.sensors.canandmag.CanandmagSettings;
import edu.wpi.first.math.geometry.Rotation2d;

public class AbsoluteEncoderIORedux implements AbsoluteEncoderIO {

  private final Canandmag encoder;
  private final CanandmagSettings settings;
  private final double offset;

  public AbsoluteEncoderIORedux(
      int canID, AbsoluteEncoderConfig config, boolean disableZeroButton) {
    encoder = new Canandmag(canID);
    settings =
        new CanandmagSettings()
            .setInvertDirection(config.invert())
            .setDisableZeroButton(disableZeroButton);
    offset = config.offsetRot();

    encoder.setSettings(settings);
  }

  @Override
  public void updateInputs(AbsoluteEncoderIOInputs inputs) {
    inputs.connected = encoder.isConnected(1);
    inputs.angle = Rotation2d.fromRotations(encoder.getAbsPosition() + offset);
  }

  @Override
  public double getNoOffsetAngle() {
    return encoder.getAbsPosition();
  }
}
