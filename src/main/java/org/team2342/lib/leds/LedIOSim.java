// Copyright (c) 2025 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.leds;

import org.littletonrobotics.junction.Logger;

public class LedIOSim implements LedIO {
  private LedColor firstColor = LedColor.off();
  private LedColor secondColor = LedColor.off();
  private LedEffect firstEffect = LedEffect.OFF;
  private LedEffect secondEffect = LedEffect.OFF;

  @Override
  public void setColor(Half half, LedColor color) {
    if (color == null) {
      color = LedColor.off();
    }

    switch (half) {
      case FIRST:
        firstColor = color;
        firstEffect = LedEffect.SOLID;
        break;
      case SECOND:
        secondColor = color;
        secondEffect = LedEffect.SOLID;
        break;
      case ALL:
        firstColor = color;
        secondColor = color;
        firstEffect = LedEffect.SOLID;
        secondEffect = LedEffect.SOLID;
        break;
    }
  }

  @Override
  public void setEffect(Half half, LedEffect effect, LedColor color){
    if (color == null) {
      color = LedColor.off();
    }

    switch (half) {
      case FIRST:
        firstColor = color;
        firstEffect = effect;
        break;
      case SECOND:
        secondColor = color;
        secondEffect = effect;
        break;
      case ALL:
        firstColor = color;
        secondColor = color;
        firstEffect = effect;
        secondEffect = effect;
        break;
    }
  }

  @Override
  public void updateInputs(LedIOInputs inputs){
    inputs.firstHalfColor = firstColor;
    inputs.secondHalfColor = secondColor;
    inputs.firstHalfEffect = firstEffect;
    inputs.secondHalfEffect = secondEffect;

    Logger.recordOutput("LED/FirstHalf/Red", firstColor.red);
    Logger.recordOutput("LED/FirstHalf/Green", firstColor.green);
    Logger.recordOutput("LED/FirstHalf/Blue", firstColor.blue);
    Logger.recordOutput("LED/FirstHalf/Effect", firstEffect.toString());

    Logger.recordOutput("LED/SecondHalf/Red", secondColor.red);
    Logger.recordOutput("LED/SecondHalf/Green", secondColor.green);
    Logger.recordOutput("LED/SecondHalf/Blue", secondColor.blue);
    Logger.recordOutput("LED/SecondHalf/Effect", secondEffect.toString());
  }
}