// Copyright (c) 2025 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.leds;

import com.ctre.phoenix.led.Animation;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Interface for LED input/output Allows for different LED implementations to use the same structure
 */
public interface LedIO {
  /** Container class for LED inputs - used for logging and data updates. */
  public static class LedIOInputs {
    public Color firstHalfColor = Color.kWhite;
    public Color secondHalfColor = Color.kWhite;
  }

  /**
   * Called periodically to update the LED input data.
   *
   * @param inputs The object that stores LED readings to be logged or used somewhere else.
   */
  public default void updateInputs(LedIOInputs inputs) {}
  /**
   * Sets the color for the first or second half of the LED strip.
   *
   * @param color The color to set for the half
   */
  public default void setFirstHalfColor(Color color) {}

  public default void setSecondHalfColor(Color color) {}
  /**
   * Sets the animation for the LED strip.
   *
   * @param animation The animation to apply to the LED strip.
   */
  public default void setAnimation(Animation animation) {}
}
