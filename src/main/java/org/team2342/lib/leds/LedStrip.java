// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team2342.lib.leds.LedIO.LedEffect;

public class LedStrip extends SubsystemBase {
  private final LedIOCANdle io;

  public LedStrip(LedIOCANdle io, String name) {
    this.io = io;
  }

  public void setFirst(LedEffect effect, Color color) {
    io.setEffect(LedIO.Half.FIRST, effect, color);
  }

  public void setSecond(LedEffect effect, Color color) {
    io.setEffect(LedIO.Half.SECOND, effect, color);
  }

  public void setAll(LedEffect effect, Color color) {
    setFirst(effect, color);
    setSecond(effect, color);
  }
}
