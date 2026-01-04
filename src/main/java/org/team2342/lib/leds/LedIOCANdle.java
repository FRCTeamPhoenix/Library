// Copyright (c) 2025 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.leds;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;


public class LedIOCANdle implements LedIO {
  private CANdle candle; 
  private int totalLeds;
  private int firstHalfLength;
  private int secondHalfLength; 
  
  private LedColor lastFirstColor = LedColor.off();
  private LedColor lastSecondColor = LedColor.off();
  private LedEffect lastFirstEffect = LedEffect.OFF;
  private LedEffect lastSecondEffect = LedEffect.OFF;

  public LedIOCANdle(int canID, int ledCount){
    this.candle = new CANdle(canID);
    this.totalLeds = ledCount;
    this.firstHalfLength = totalLeds / 2;
    this.secondHalfLength = totalLeds - firstHalfLength;

    CANdleConfiguration config = new CANdleConfiguration();
    config.LED.StripType = StripTypeValue.RGB;
    candle.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(LedIOInputs inputs){
    inputs.firstHalfColor = lastFirstColor;
    inputs.secondHalfColor = lastSecondColor;
    inputs.firstHalfEffect = lastFirstEffect;
    inputs.secondHalfEffect = lastSecondEffect;
  }
  
  @Override
  public void setColor(Half half, LedColor color) {
    switch (half) {
      case FIRST:
        sendSolidColor(0, firstHalfLength, color);
        lastFirstColor = color;
        lastFirstEffect = LedEffect.SOLID;
        break;
      case SECOND:
        sendSolidColor(firstHalfLength, secondHalfLength, color);
        lastSecondColor = color;
        lastSecondEffect = LedEffect.SOLID;
        break;
      case ALL:
        sendSolidColor(0, totalLeds, color);
        lastFirstColor = color;
        lastSecondColor = color;
        lastFirstEffect = LedEffect.SOLID;
        lastSecondEffect = LedEffect.SOLID;
        break;
      }

  }

  @Override
  public void setEffect(Half half, LedEffect effect, LedColor color) {
    if (color == null){
      color = LedColor.off();
    }

    int start = 0;
    int length = 0;

    switch (half) {
      case FIRST:
        start = 0;
        length = firstHalfLength;
        lastFirstColor = color;
        lastFirstEffect = effect;
        break;
      case SECOND:
        start = firstHalfLength;
        length = secondHalfLength;
        lastSecondColor = color;
        lastSecondEffect = effect;
        break;
      case ALL:
        start = 0;
        length = totalLeds;
        lastFirstEffect = effect;
        lastSecondEffect = effect;
        lastFirstColor = color;
        lastSecondColor = color;
        break;
    }

    switch (effect) {
      case SOLID:
        sendSolidColor(start, length, color);
        break;
      case RAINBOW:
        RainbowAnimation rainbow = new RainbowAnimation(start, length);
        candle.setControl(rainbow);
        break;
      case FLASHING:
        sendSolidColor(start, length, color);
        break;
      case OFF:
        sendSolidColor(start, length, LedColor.off());
        break;
    }
  }

  private RGBWColor toCTRE(LedColor c) {
    if (c == null){
      return new RGBWColor(0,0,0,0);
    }
    return new RGBWColor(c.red, c.green, c.blue,0);
  }

  private void sendSolidColor(int start, int length, LedColor color) {
    SolidColor request = new SolidColor(start, length);
    request.withColor(toCTRE(color));
    candle.setControl(request);
  }
}