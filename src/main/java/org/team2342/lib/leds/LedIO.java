// Copyright (c) 2025 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.leds;

public interface LedIO {

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

  public static class LedColor {
    public int red;
    public int green;
    public int blue;

    public LedColor(int red, int green, int blue) {
      this.red = red;
      this.green = green;
      this.blue = blue;
    }

    public static LedColor off() {
      return new LedColor(0, 0, 0);
    }
  }

  public static class LedIOInputs {
    public LedColor firstHalfColor = LedColor.off();
    public LedColor secondHalfColor = LedColor.off();
    public LedEffect firstHalfEffect = LedEffect.OFF;
    public LedEffect secondHalfEffect = LedEffect.OFF;
  }

  public default void updateInputs(LedIOInputs inputs) {}

  public default void setColor(Half half, LedColor color) {}

  public default void setEffect(Half half, LedEffect effect, LedColor color) {}

  public default void setAllColor(LedColor color) {
    setColor(Half.FIRST, color);
    setColor(Half.SECOND, color);
  }

  public default void setAllEffect(LedEffect effect, LedColor color) {
    setEffect(Half.FIRST, effect, color);
    setEffect(Half.SECOND, effect, color);
  }
}
