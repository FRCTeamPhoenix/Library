// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.leds;

import edu.wpi.first.wpilibj.util.Color;
import org.littletonrobotics.junction.AutoLog;

public interface LedIO {
  @AutoLog
  public static class LedIOInputs {
    public Color firstHalfColor = Color.kBlack;
    public Color secondHalfColor = Color.kBlack;
    public LedEffect firstHalfEffect = LedEffect.OFF;
    public LedEffect secondHalfEffect = LedEffect.OFF;
  }

  public default void updateInputs(LedIOInputs inputs) {}

  public default void setColor(Half half, Color color) {}

  public default void setEffect(Half half, LedEffect effect, Color color) {}

  public default void setColor(Color color) {
    setColor(Half.ALL, color);
  }

  public default void setEffect(LedEffect effect, Color color) {
    setEffect(Half.ALL, effect, color);
  }

  public enum Half {
    FIRST,
    SECOND,
    ALL
  }

  public enum LedEffect {
    SOLID,
    FLASHING,
    RAINBOW,
    OFF
  }
}
