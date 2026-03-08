// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.leds;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team2342.lib.leds.LedIO.LEDEffect;
import org.team2342.lib.logging.ExecutionLogger;

public class LedStrip extends SubsystemBase {
  private final LedIO io;
  private final String name;
  private final LedIOInputsAutoLogged inputs = new LedIOInputsAutoLogged();

  public LedStrip(LedIO io, String name) {
    this.io = io;
    this.name = name;
    setName(name);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    ExecutionLogger.log(name);
  }

  public void setFirst(LEDEffect effect) {
    io.setEffect(LedIO.Half.FIRST, effect);
  }

  public void setSecond(LEDEffect effect) {
    io.setEffect(LedIO.Half.SECOND, effect);
  }

  public void setAll(LEDEffect effect) {
    setFirst(effect);
    setSecond(effect);
  }
}
