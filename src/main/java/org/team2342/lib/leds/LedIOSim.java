// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.leds;

import edu.wpi.first.wpilibj.util.Color;

public class LedIOSim implements LedIO {
  private Color firstColor = Color.kBlack;
  private Color secondColor = Color.kBlack;
  private LedEffect firstEffect = LedEffect.OFF;
  private LedEffect secondEffect = LedEffect.OFF;

  @Override
  public void setColor(Half half, Color color) {
    switch (half) {
      case FIRST:
        firstColor = color;
        break;
      case SECOND:
        secondColor = color;
        break;
      case ALL:
        firstColor = color;
        secondColor = color;
    }
  }

  @Override
  public void setEffect(Half half, LedEffect effect, Color color) {
    setColor(half, color);

    switch (half) {
      case FIRST:
        firstEffect = effect;
        break;
      case SECOND:
        secondEffect = effect;
        break;
      case ALL:
        firstEffect = effect;
        secondEffect = effect;
        break;
    }
  }

  // @Override
  // public void updateInputs(LedIOInputs inputs) {
  //   inputs.firstHalfColor = firstColor;
  //   inputs.secondHalfColor = secondColor;
  //   inputs.firstHalfEffect = firstEffect;
  //   inputs.secondHalfEffect = secondEffect;

  //   Logger.recordOutput(
  //       "LED/FirstHalf/Color", new double[] {firstColor.red, firstColor.green, firstColor.blue});
  //   Logger.recordOutput("LED/FirstHalf/Effect", firstEffect.name());

  //   Logger.recordOutput(
  //       "LED/SecondHalf/Color",
  //       new double[] {secondColor.red, secondColor.green, secondColor.blue});
  //   Logger.recordOutput("LED/SecondHalf/Effect", secondEffect.name());
  // }
}
