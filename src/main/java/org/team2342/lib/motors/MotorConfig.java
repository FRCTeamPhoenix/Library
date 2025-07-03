// Copyright (c) 2025 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.motors;

public class MotorConfig {

  public boolean motorInverted = false;
  public double supplyLimit = 0;
  public double statorLimit = 0;
  public IdleMode idleMode = IdleMode.COAST;

  public MotorConfig() {}

  public MotorConfig withMotorInverted(boolean inverted) {
    motorInverted = inverted;
    return this;
  }

  public MotorConfig withSupplyCurrentLimit(double limit) {
    supplyLimit = limit;
    return this;
  }

  public MotorConfig withStatorCurrentLimit(double limit) {
    statorLimit = limit;
    return this;
  }

  public MotorConfig withIdleMode(IdleMode mode) {
    idleMode = mode;
    return this;
  }

  public enum IdleMode {
    /** Go into brake mode when no output is being applied */
    BRAKE,
    /** Coast when no output is being applied */
    COAST
  }
}
