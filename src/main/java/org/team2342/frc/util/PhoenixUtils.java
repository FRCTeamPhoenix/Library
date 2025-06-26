// Copyright (c) 2025 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import java.util.function.Supplier;

public class PhoenixUtils {
  // All registered signals
  private static BaseStatusSignal[] registeredSignals = new BaseStatusSignal[0];

  /**
   * Try given command until an OK signal is given, or until the max attempts is reached.
   *
   * @param maxAttempts Max amount of times to try the command
   * @param command Supplier for the command to try. Needs to return {@link
   *     com.ctre.phoenix6.StatusCode}
   */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) break;
    }
  }

  /**
   * Register signals ({@link com.ctre.phoenix6.BaseStatusSignal}) to be refreshed every periodic
   * loop.
   *
   * @param signals Signals to register
   */
  public static void registerSignals(BaseStatusSignal... signals) {
    // Need to keep signals in an array, so array copies are necessary
    BaseStatusSignal[] newSignals = new BaseStatusSignal[registeredSignals.length + signals.length];
    System.arraycopy(registeredSignals, 0, newSignals, 0, registeredSignals.length);
    System.arraycopy(signals, 0, newSignals, registeredSignals.length, signals.length);
    registeredSignals = newSignals;
  }

  /** Refresh all registered signals. */
  public static void refreshSignals() {
    // Check to make sure there are signals to refresh
    if (registeredSignals.length > 0) BaseStatusSignal.refreshAll(registeredSignals);
  }
}
