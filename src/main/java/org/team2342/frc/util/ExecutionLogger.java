// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc.util;

import org.littletonrobotics.junction.Logger;
import org.wpilib.system.RobotController;

/** Class to log code execution times */
public class ExecutionLogger {

  private static double lastNS = 0.0;

  /** Reset time. */
  public static void reset() {
    lastNS = RobotController.getMonotonicTime();
  }

  /** Log execution time under the given name. */
  public static void log(String name) {
    long currentNS = RobotController.getMonotonicTime();
    Logger.recordOutput(
        String.format("ExecutionLogger/%sMS", name), (currentNS - lastNS) / 1_000_000.0);
    lastNS = currentNS;
  }
}
