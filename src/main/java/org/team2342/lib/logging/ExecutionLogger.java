// Copyright (c) 2025 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.logging;

import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

/** Class to log code execution times */
public class ExecutionLogger {

  private static double lastMS = 0.0;

  /** Reset time. */
  public static void reset() {
    lastMS = Timer.getFPGATimestamp() * 1000.0;
  }

  /** Log execution time under the given name. */
  public static void log(String name) {
    double currentMS = Timer.getFPGATimestamp() * 1000.0;
    Logger.recordOutput(String.format("ExecutionLogger/%sMS", name), currentMS - lastMS);
    lastMS = currentMS;
  }
}
