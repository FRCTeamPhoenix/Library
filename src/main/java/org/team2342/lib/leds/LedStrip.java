// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team2342.lib.leds.LedIO.LedEffect;
import org.team2342.lib.leds.LedIO.LedIOInputs;

public class LedStrip extends SubsystemBase {
  private final LedIO io;
  private final LedIOInputs inputs = new LedIOInputs();

  public LedStrip(LedIO io) {
    this.io = io;
    setName("Leds");
  }

  public void setDriver(Color color, LedEffect effect) {
    io.setEffect(LedIO.Half.FIRST, effect, color);
  }

  public void setOperator(Color color, LedEffect effect) {
    io.setEffect(LedIO.Half.SECOND, effect, color);
  }

  public void setAll(Color color, LedEffect effect) {
    io.setEffect(effect, color);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }
}
