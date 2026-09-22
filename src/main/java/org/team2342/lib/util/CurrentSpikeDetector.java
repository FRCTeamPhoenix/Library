// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.util;

import java.util.function.BooleanSupplier;
import org.wpilib.command2.button.Trigger;
import org.wpilib.system.Timer;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Time;

public class CurrentSpikeDetector implements BooleanSupplier {
  private Current currentThreshold;
  private Time timeThreshold;
  private Timer thresholdTimer;
  private Trigger trigger;

  private boolean spikeDetected = false;

  public CurrentSpikeDetector(Current currentThresholdAmps, Time timeThresholdSeconds) {
    this.currentThreshold = currentThresholdAmps;
    this.timeThreshold = timeThresholdSeconds;
    this.thresholdTimer = new Timer();
  }

  public boolean update(Current current) {
    if (current.gt(currentThreshold)) {
      thresholdTimer.start();
      boolean spikeFound = thresholdTimer.hasElapsed(timeThreshold);
      spikeDetected = spikeFound;
    } else {
      thresholdTimer.stop();
      thresholdTimer.reset();
      spikeDetected = false;
    }
    return spikeDetected;
  }

  @Override
  public boolean getAsBoolean() {
    return spikeDetected;
  }

  public Trigger asTrigger() {
    if (trigger == null) trigger = new Trigger(this);
    return trigger;
  }
}
