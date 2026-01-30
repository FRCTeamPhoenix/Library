// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.leds;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.Enable5VRailValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj.util.Color;
import org.team2342.frc.util.PhoenixUtils;

public class LedIOCANdle implements LedIO {
  private final CANdle candle;
  private final int halfLength;
  // private int startFirst = 0;
  // private int endFirst;
  // private int startSecond;
  // private int endSecond;

  private final int slot0StartIdx;
  private final int slot0EndIdx;
  private final int slot1StartIdx;
  private final int slot1EndIdx;
  private final int candleFirstStart = 0;
  private final int canddleFirstEnd = 3;
  private final int candleSecondStart = 4;
  private final int candleSecondEnd = 7;
  private Color firstColor = Color.kBlack;
  private Color secondColor = Color.kBlack;
  private LedEffect firstEffect = LedEffect.OFF;
  private LedEffect secondEffect = LedEffect.OFF;

  public LedIOCANdle(int canId, int ledCount) {
    this.candle = new CANdle(canId, new CANBus("rio"));
    this.halfLength = ledCount / 2;
    slot0StartIdx = 8;
    slot0EndIdx = this.halfLength;
    slot1StartIdx = slot0EndIdx + 1;
    slot1EndIdx = ledCount;

    CANdleConfiguration config = new CANdleConfiguration();
    config.LED.StripType = StripTypeValue.GRB;
    config.LED.BrightnessScalar = 0.7;
    config.CANdleFeatures.Enable5VRail = Enable5VRailValue.Enabled;
    // config.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
    candle.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(LedIOInputs inputs) {
    inputs.firstHalfColor = firstColor;
    inputs.secondHalfColor = secondColor;
    inputs.firstHalfEffect = firstEffect;
    inputs.secondHalfEffect = secondEffect;
  }

  // @Override
  // public void setColor(Half half, Color color) {
  //   if (color == null) {
  //     color = Color.kBlack;
  //   }

  //   switch (half) {
  //     case FIRST -> {
  //       candle.setControl(
  //         new SolidColor(slot0StartIdx, slot0EndIdx).withColor(PhoenixUtils.toCTREColor(color))
  //       );
  //     }
  //     case SECOND -> {
  //       candle.setControl(
  //         new SolidColor(slot1StartIdx, slot1EndIdx).withColor(PhoenixUtils.toCTREColor(color))
  //       );
  //     }
  //     // case ALL -> {
  //     //   sendSolidColor(slot0StartIdx, slot1EndIdx, color);
  //     // }
  //   }
  // }

  @Override
  public void setEffect(Half half, LedEffect effect, Color color) {
    if (color == null) {
      color = Color.kBlack;
    }
    switch (half) {
      case FIRST -> {
        firstColor = color;
        firstEffect = effect;
        candle.setControl(
            new SolidColor(candleFirstStart, canddleFirstEnd)
                .withColor(PhoenixUtils.toCTREColor(color)));
        switch (effect) {
          case SOLID -> {
            candle.setControl(
                new SolidColor(slot0StartIdx, slot0EndIdx)
                    .withColor(PhoenixUtils.toCTREColor(color)));
          }
          case FLASHING -> {
            candle.setControl(
                new StrobeAnimation(slot0StartIdx, slot0EndIdx)
                    .withSlot(0)
                    .withColor(PhoenixUtils.toCTREColor(color)));
          }
          case FIRE -> {
            candle.setControl(new FireAnimation(slot0StartIdx, slot0EndIdx).withSlot(0));
          }
          case TWINKLE -> {
            candle.setControl(
                new TwinkleAnimation(slot0StartIdx, slot0EndIdx)
                    .withSlot(0)
                    .withColor(PhoenixUtils.toCTREColor(color)));
          }
          case COLOR_FLOW -> {
            candle.setControl(
                new ColorFlowAnimation(slot0StartIdx, slot0EndIdx)
                    .withSlot(0)
                    .withColor(PhoenixUtils.toCTREColor(color)));
          }
          case LARSON -> {
            candle.setControl(
                new LarsonAnimation(slot0StartIdx, slot0EndIdx)
                    .withSlot(0)
                    .withColor(PhoenixUtils.toCTREColor(color)));
          }
          case RGB_FADE -> {
            candle.setControl(new RgbFadeAnimation(slot0StartIdx, slot0EndIdx).withSlot(0));
          }
          case RAINBOW -> {
            candle.setControl(new RainbowAnimation(slot0StartIdx, slot0EndIdx).withSlot(0));
          }
          case OFF -> {
            candle.setControl(
                new SolidColor(slot0StartIdx, slot0EndIdx)
                    .withColor(PhoenixUtils.toCTREColor(Color.kBlack)));
          }
        }
      }
      case SECOND -> {
        secondColor = color;
        secondEffect = effect;
        candle.setControl(
            new SolidColor(candleSecondStart, candleSecondEnd)
                .withColor(PhoenixUtils.toCTREColor(color)));
        switch (effect) {
          case SOLID -> {
            candle.setControl(
                new SolidColor(slot1StartIdx, slot1EndIdx)
                    .withColor(PhoenixUtils.toCTREColor(color)));
          }
          case FLASHING -> {
            candle.setControl(
                new StrobeAnimation(slot1StartIdx, slot1EndIdx)
                    .withSlot(1)
                    .withColor(PhoenixUtils.toCTREColor(color)));
          }
          case FIRE -> {
            candle.setControl(new FireAnimation(slot1StartIdx, slot1EndIdx).withSlot(1));
          }
          case TWINKLE -> {
            candle.setControl(
                new TwinkleAnimation(slot1StartIdx, slot1EndIdx)
                    .withSlot(1)
                    .withColor(PhoenixUtils.toCTREColor(color)));
          }
          case COLOR_FLOW -> {
            candle.setControl(
                new ColorFlowAnimation(slot1StartIdx, slot1EndIdx)
                    .withSlot(1)
                    .withColor(PhoenixUtils.toCTREColor(color)));
          }
          case LARSON -> {
            candle.setControl(
                new LarsonAnimation(slot1StartIdx, slot1EndIdx)
                    .withSlot(1)
                    .withColor(PhoenixUtils.toCTREColor(color)));
          }
          case RGB_FADE -> {
            candle.setControl(new RgbFadeAnimation(slot1StartIdx, slot1EndIdx).withSlot(1));
          }
          case RAINBOW -> {
            candle.setControl(new RainbowAnimation(slot1StartIdx, slot1EndIdx).withSlot(1));
          }
          case OFF -> {
            candle.setControl(
                new SolidColor(slot1StartIdx, slot1EndIdx)
                    .withColor(PhoenixUtils.toCTREColor(Color.kBlack)));
          }
        }
      }
        // case ALL -> {
        //   firstColor = color;
        //   secondColor = color;
        //   firstEffect = effect;
        //   secondEffect = effect;
        // }
    }
  }

  public void clearAll() {
    for (int i = 0; i < 8; ++i) {
      candle.setControl(new EmptyAnimation(i));
    }
  }
  // private void applyEffect(int start, int end, LedEffect effect, Color color) {
  //   switch (effect) {
  //     case SOLID -> {
  //       sendSolidColor(start, end, color);
  //     }
  //     case FLASHING -> {
  //
  //     }
  //     case RAINBOW -> {
  //       // candle.setControl(new EmptyAnimation(end));
  //       RainbowAnimation request = new RainbowAnimation(start, end).withSlot(0);
  //       candle.setControl(request);
  //     }
  //     case OFF -> {
  //       // candle.setControl(new EmptyAnimation(end));
  //       sendSolidColor(start, end, Color.kBlack);
  //     }
  //   }
  // }

}
