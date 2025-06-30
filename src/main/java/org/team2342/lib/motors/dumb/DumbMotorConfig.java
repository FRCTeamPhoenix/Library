// Copyright (c) 2025 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.motors.dumb;

public class DumbMotorConfig {

  public boolean motorInverted = false;
  public double supplyLimit = 0;
  public double statorLimit = 0;
  public IdleMode idleMode = IdleMode.COAST;

  public DumbMotorConfig() {}

  public DumbMotorConfig withMotorInverted(boolean inverted) {
    motorInverted = inverted;
    return this;
  }

  public DumbMotorConfig withSupplyCurrentLimit(double limit) {
    supplyLimit = limit;
    return this;
  }

  public DumbMotorConfig withStatorCurrentLimit(double limit) {
    statorLimit = limit;
    return this;
  }

  public DumbMotorConfig withIdleMode(IdleMode mode) {
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
