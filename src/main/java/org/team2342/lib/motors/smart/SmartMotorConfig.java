// Copyright (c) 2025 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.motors.smart;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import org.team2342.lib.motors.MotorConfig;
import org.team2342.lib.pidff.PIDFFConfigs;

public class SmartMotorConfig extends MotorConfig {

  public PIDFFConfigs pidffConfigs = new PIDFFConfigs();
  public double gearRatio = 1;
  public ControlType controlType = null;

  public Constraints profileConstraintsRad = new Constraints(0, 0);

  /**
   * This is meant for when you use WPILib Sim Classes that don't report directly in radians, such
   * as the ElevatorSim. For most use cases, this doesn't need to be changed
   */
  public double simRatio = 1.0;

  public SmartMotorConfig() {}

  public SmartMotorConfig withPIDFFConfigs(PIDFFConfigs configs) {
    pidffConfigs = configs;
    return this;
  }

  public SmartMotorConfig withGearRatio(double gearRatio) {
    this.gearRatio = gearRatio;
    return this;
  }

  public SmartMotorConfig withControlType(ControlType type) {
    controlType = type;
    return this;
  }

  public SmartMotorConfig withProfileConstraintsRad(Constraints constraints) {
    this.profileConstraintsRad = constraints;
    return this;
  }

  /**
   * This is meant for when you use WPILib Sim Classes that don't report directly in radians, such
   * as the ElevatorSim. For most use cases, this doesn't need to be changed
   */
  public SmartMotorConfig withSimRatio(double ratio) {
    simRatio = ratio;
    return this;
  }

  @Override
  public SmartMotorConfig withMotorInverted(boolean inverted) {
    motorInverted = inverted;
    return this;
  }

  @Override
  public SmartMotorConfig withSupplyCurrentLimit(double limit) {
    supplyLimit = limit;
    return this;
  }

  @Override
  public SmartMotorConfig withStatorCurrentLimit(double limit) {
    statorLimit = limit;
    return this;
  }

  @Override
  public SmartMotorConfig withIdleMode(IdleMode mode) {
    idleMode = mode;
    return this;
  }

  public enum ControlType {
    /** Velocity closed-loop control */
    VELOCITY,
    /** Position closed-loop control */
    POSITION,
    /** Position control with velocity and acceleration limits */
    PROFILED_POSITION,
    /** Velocity control, with acceleration limits */
    PROFILED_VELOCITY,
  }

  public record FollowerConfig(int canID, boolean inverted) {}
}
