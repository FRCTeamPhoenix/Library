// Copyright (c) 2025 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedStrip extends SubsystemBase {
  private final LedIO io;
  private LedIO.LedColor firstColor = LedIO.LedColor.off();
  private LedIO.LedEffect firstEffect = LedIO.LedEffect.OFF;
  private LedIO.LedColor secondColor = LedIO.LedColor.off();
  private LedIO.LedEffect secondEffect = LedIO.LedEffect.OFF;

  private boolean flashing = false;
  private double lastFlashTime = 0.0;
  private static final double FLASH_PERIOD_SEC = 0.5;

  public LedStrip(LedIO io){
    this.io = io;
  }

  public void setFirst(LedIO.LedEffect effect, LedIO.LedColor color){
    firstEffect = effect;
    firstColor = color;
  }

  public void setSecond(LedIO.LedEffect effect, LedIO.LedColor color){
    secondEffect = effect;
    secondColor = color;
  }

  @Override
  public void periodic() {
    updateFlashing();
    applyEffect(LedIO.Half.FIRST, firstEffect, firstColor);
    applyEffect(LedIO.Half.SECOND, secondEffect, secondColor);
  }

  private void updateFlashing() {
    double now = Timer.getFPGATimestamp();
    if (now - lastFlashTime >= FLASH_PERIOD_SEC) {
      flashing = !flashing;
      lastFlashTime = now;
    }
  }

  private void applyEffect(LedIO.Half half, LedIO.LedEffect effect, LedIO.LedColor color){
    switch (effect) {
      case SOLID:
        io.setColor(half, color);
        break;
      case FLASHING:
        io.setColor(half, (flashing ? color : LedIO.LedColor.off()));
        break;
      case RAINBOW:
        io.setEffect(half, LedIO.LedEffect.RAINBOW, color);
        break;
      case OFF:
        io.setColor(half, LedIO.LedColor.off());
        break;
    }
  }
}