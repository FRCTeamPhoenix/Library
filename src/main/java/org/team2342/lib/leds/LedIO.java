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
    public LEDAnimation firstHalfEffect = LEDAnimation.OFF;
    public LEDAnimation secondHalfEffect = LEDAnimation.OFF;
  }

  public default void updateInputs(LedIOInputs inputs) {}

  public default void setEffect(Half half, LEDEffect effect) {}

  public enum Half {
    FIRST,
    SECOND,
  }

  public enum LEDAnimation {
    SOLID,
    FLASHING,
    FIRE,
    TWINKLE,
    COLOR_FLOW,
    LARSON,
    RGB_FADE,
    RAINBOW,
    OFF
  }

  public record LEDEffect(LEDAnimation animation, Color color) {}
}
