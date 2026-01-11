// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team2342.lib.leds.LedIO.LedEffect;
// import org.team2342.lib.leds.LedIOInputsAutoLogged;

public class LedStrip extends SubsystemBase {
  private final LedIOCANdle io;
  private final String name;
  // private final LedIOInputsAutoLogged inputs = new LedIOInputsAutoLogged();

  public LedStrip(LedIOCANdle io, String name) {
    this.io = io;
    this.name = name;
    setName(name);
  }

  public void setFirst(Color color, LedEffect effect) {
    io.setEffect(LedIO.Half.FIRST, effect, color);
  }

  public void setSecond(Color color, LedEffect effect) {
    io.setEffect(LedIO.Half.SECOND, effect, color);
  }

  public void setAll(Color color, LedEffect effect) {
    io.setEffect(LedIO.Half.ALL, effect, color);
  }

  // @Override
  // public void periodic() {
  //   io.updateInputs(inputs);
  //   Logger.processInputs(name, inputs);
  // }
}
