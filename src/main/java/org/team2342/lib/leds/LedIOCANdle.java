// Copyright (c) 2025 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.leds;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Implementation of LedIO using a CANdle device. This class manages the LED strip by controlling
 * the colors and animations.
 */
public class LedIOCANdle implements LedIO {
  private final CANdle candle;
  private final int totalLeds;
  private final int halfLength;

  private Color firstHalfColor = Color.kWhite;
  private Color secondHalfColor = Color.kWhite;

  /**
   * Constructor for LedIOCANdle. Initializes the CANdle with the specified CAN ID and LED count.
   *
   * @param canId The CAN ID of the CANdle device.
   * @param LedCount The total number of LEDs in the strip.
   */
  public LedIOCANdle(int canId, int LedCount) {
    candle = new CANdle(canId);
    totalLeds = LedCount;
    halfLength = LedCount / 2;

    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB;
    config.brightnessScalar = 1.0;
    candle.configAllSettings(config);
  }

  /**
   * Called periodically to update the LED input data.
   *
   * @param inputs The LedIOInputs object to update
   */
  @Override
  public void updateInputs(LedIOInputs inputs) {
    inputs.firstHalfColor = firstHalfColor;
    inputs.secondHalfColor = secondHalfColor;
  }

  /**
   * Sets the color for the first half and second half of the LED strip.
   *
   * @param color The color to set for the half
   */
  @Override
  public void setFirstHalfColor(Color color) {
    firstHalfColor = color;
    candle.clearAnimation(0);
    candle.setLEDs(
        (int) (color.red * 255),
        (int) (color.green * 255),
        (int) (color.blue * 255),
        0,
        0,
        halfLength);
  }

  @Override
  public void setSecondHalfColor(Color color) {
    secondHalfColor = color;
    candle.clearAnimation(0);
    candle.setLEDs(
        (int) (color.red * 255),
        (int) (color.green * 255),
        (int) (color.blue * 255),
        0,
        halfLength,
        halfLength);
  }

  /**
   * Sets the animation for the LED strip.
   *
   * @param animation The animation to apply to the LED strip.
   */
  @Override
  public void setAnimation(Animation animation) {
    candle.clearAnimation(0);
    animation.setNumLed(totalLeds);
    candle.animate(animation);
  }
}
