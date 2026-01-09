// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.leds;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj.util.Color;
import org.team2342.frc.util.PhoenixUtils;

public class LedIOCANdle implements LedIO {
  private final CANdle candle;
  private final int ledCount;
  private final int halfLength;

  private Color firstColor = Color.kBlack;
  private Color secondColor = Color.kBlack;
  private LedEffect firstEffect = LedEffect.OFF;
  private LedEffect secondEffect = LedEffect.OFF;

  public LedIOCANdle(int canId, int ledCount) {
    this.candle = new CANdle(canId);
    this.ledCount = ledCount;
    this.halfLength = ledCount / 2;

    CANdleConfiguration config = new CANdleConfiguration();
    config.LED.StripType = StripTypeValue.RGB;
    candle.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(LedIOInputs inputs) {
    inputs.firstHalfColor = firstColor;
    inputs.secondHalfColor = secondColor;
    inputs.firstHalfEffect = firstEffect;
    inputs.secondHalfEffect = secondEffect;
  }

  @Override
  public void setColor(Half half, Color color) {
    switch (half) {
      case FIRST -> {
        sendSolidColor(0, halfLength, color);
        firstColor = color;
        firstEffect = LedEffect.SOLID;
      }
      case SECOND -> {
        sendSolidColor(halfLength, ledCount, color);
        secondColor = color;
        secondEffect = LedEffect.SOLID;
      }
      case ALL -> {
        sendSolidColor(0, ledCount, color);
        firstColor = color;
        secondColor = color;
        firstEffect = LedEffect.SOLID;
        secondEffect = LedEffect.SOLID;
      }
    }
  }

  @Override
  public void setEffect(Half half, LedEffect effect, Color color) {
    if (color == null) {
      color = Color.kBlack;
    }

    switch (half) {
      case FIRST:
        firstColor = color;
        firstEffect = effect;
        applyEffect(0, halfLength, effect, color);
        break;
      case SECOND:
        secondColor = color;
        secondEffect = effect;
        applyEffect(halfLength, ledCount, effect, color);
        break;
      case ALL:
        firstColor = color;
        secondColor = color;
        firstEffect = effect;
        secondEffect = effect;
        break;
    }
  }

  private void sendSolidColor(int start, int end, Color color) {
    SolidColor request = new SolidColor(start, end);
    request.withColor(PhoenixUtils.toCTREColor(color));
    candle.setControl(request);
  }

  private void applyEffect(int start, int end, LedEffect effect, Color color) {
    switch (effect) {
      case SOLID -> {
        sendSolidColor(start, end, color);
      }
      case FLASHING -> {
        StrobeAnimation request = new StrobeAnimation(start, end);
        request.withColor(PhoenixUtils.toCTREColor(color));
        candle.setControl(request);
      }
      case RAINBOW -> {
        RainbowAnimation request = new RainbowAnimation(start, end);
        candle.setControl(request);
      }
      case OFF -> {
        sendSolidColor(start, end, Color.kBlack);
      }
    }
  }
}
