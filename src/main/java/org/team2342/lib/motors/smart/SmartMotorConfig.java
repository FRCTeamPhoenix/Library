// Copyright (c) 2025 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.motors.smart;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import org.team2342.lib.pidff.PIDFFConfigs;

public class SmartMotorConfig {

  public PIDFFConfigs pidffConfigs = new PIDFFConfigs();
  public double gearRatio = 1;
  public boolean motorInverted = false;
  public double supplyLimit = 0;
  public double statorLimit = 0;
  public ControlType controlType = null;
  public IdleMode idleMode = IdleMode.COAST;

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

  public SmartMotorConfig withMotorInverted(boolean inverted) {
    motorInverted = inverted;
    return this;
  }

  public SmartMotorConfig withSupplyCurrentLimit(double limit) {
    supplyLimit = limit;
    return this;
  }

  public SmartMotorConfig withStatorCurrentLimit(double limit) {
    statorLimit = limit;
    return this;
  }

  public SmartMotorConfig withControlType(ControlType type) {
    controlType = type;
    return this;
  }

  public SmartMotorConfig withIdleMode(IdleMode mode) {
    idleMode = mode;
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

  public enum IdleMode {
    /** Go into brake mode when no output is being applied */
    BRAKE,
    /** Coast when no output is being applied */
    COAST
  }

  public class FollowerConfig extends Pair<Integer, Boolean> {
    public FollowerConfig(Integer canID, Boolean inverted) {
      super(canID, inverted);
    }
  }
}
