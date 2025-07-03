// Copyright (c) 2025 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.motors.dumb;

import org.littletonrobotics.junction.AutoLog;

public interface DumbMotorIO {
  @AutoLog
  public static class DumbMotorIOInputs {
    public boolean connected = false;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(DumbMotorIOInputs inputs) {}

  public default void runVoltage(double voltage) {}
}
