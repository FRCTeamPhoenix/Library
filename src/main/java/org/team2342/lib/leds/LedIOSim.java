// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.leds;

import org.littletonrobotics.junction.Logger;
import org.wpilib.util.Color;

public class LedIOSim implements LedIO {
  private Color firstColor = Color.BLACK;
  private Color secondColor = Color.BLACK;
  private LEDAnimation firstAnimation = LEDAnimation.OFF;
  private LEDAnimation secondAnimation = LEDAnimation.OFF;

  @Override
  public void setEffect(Half half, LEDEffect effect) {
    switch (half) {
      case ALL:
        firstColor = effect.color();
        firstAnimation = effect.animation();
        secondColor = effect.color();
        secondAnimation = effect.animation();
      case FIRST:
        firstColor = effect.color();
        firstAnimation = effect.animation();
        break;
      case SECOND:
        secondColor = effect.color();
        secondAnimation = effect.animation();
        break;
      default:
        break;
    }
  }

  @Override
  public void updateInputs(LedIOInputs inputs) {
    inputs.firstHalfColor = firstColor;
    inputs.secondHalfColor = secondColor;
    inputs.firstHalfEffect = firstAnimation;
    inputs.secondHalfEffect = secondAnimation;

    Logger.recordOutput(
        "LED/FirstHalf/Color", new double[] {firstColor.red, firstColor.green, firstColor.blue});
    Logger.recordOutput("LED/FirstHalf/Effect", firstAnimation.name());

    Logger.recordOutput(
        "LED/SecondHalf/Color",
        new double[] {secondColor.red, secondColor.green, secondColor.blue});
    Logger.recordOutput("LED/SecondHalf/Effect", secondAnimation.name());
  }
}
