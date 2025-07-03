// Copyright (c) 2025 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.pidff;

import lombok.Getter;
import lombok.Setter;
import org.team2342.lib.pidff.PIDFFConfigs.GravityType;

/** Custom Feedforward class that uses {@link org.team2342.lib.pidff.PIDFFConfigs} */
public class FeedforwardController {

  @Getter @Setter private PIDFFConfigs configs;

  /**
   * Creates a new feedforward controller
   *
   * @param configs The configs to use
   */
  public FeedforwardController(PIDFFConfigs configs) {
    this.configs = configs;
  }

  /**
   * Calculates the feedforward from the given parameters
   *
   * @param velocity The velocity setpoint
   * @param acceleration The acceleration setpoint
   * @param angleRad The current angle of the system, for gravity feedforward calculations
   * @return The calculated feedforward
   */
  public double calculate(double velocity, double acceleration, double angleRad) {
    return configs.kS * Math.signum(velocity)
        + configs.kV * velocity
        + configs.kA * acceleration
        + configs.kG * Math.cos(angleRad);
  }

  /**
   * Calculates the feedforward from the given parameters
   *
   * <p>If the config gravity type is set to static, this method will apply the kG term
   *
   * @param velocity The velocity setpoint
   * @param acceleration The acceleration setpoint
   * @return The calculated feedforward
   */
  public double calculate(double velocity, double acceleration) {
    if (configs.gravityType == GravityType.STATIC) {
      return calculate(velocity, acceleration, 0);
    } else {
      return calculate(velocity, acceleration, Math.PI / 2);
    }
  }

  /**
   * Calculates the feedforward from the given parameters
   *
   * <p>If the config gravity type is set to static, this method will apply the kG term
   *
   * @param velocity The velocity setpoint
   * @return The calculated feedforward
   */
  public double calculate(double velocity) {
    return calculate(velocity, 0);
  }
}
