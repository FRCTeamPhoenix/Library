// Copyright (c) 2025 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.motors.dumb;

import org.littletonrobotics.junction.AutoLog;

/*
 * Interface for dumb motor input/output
 * Lets simulation, real hardware, and different motor types to use same structure
 */

public interface DumbMotorIO {
  /**
   * Container class for motor inputs - used for logging and data updates. The @AutoLog annotation
   * automatically generates code for logging with the AdvantageKit framework.
   */
  @AutoLog
  public static class DumbMotorIOInputs {
    public boolean connected = false;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  /**
   * Called periodically to update the motor input data.
   *
   * @param inputs The object that stores motor readings to be logged or used somewhere else.
   */
  public default void updateInputs(DumbMotorIOInputs inputs) {}

  /**
   * Runs the motor at a specified voltage. This method is intended to be overridden by
   * implementations to control the motor.
   *
   * @param voltage The voltage to apply to the motor.
   */
  public default void runVoltage(double voltage) {}
}
