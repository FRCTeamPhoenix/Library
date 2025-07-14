// Copyright (c) 2025 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.motors.dumb;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

/** Simulation implementation of DumbMotorIO Uses a LinearSystemSim to simulate */
public class DumbMotorIOSim implements DumbMotorIO {
  private final LinearSystemSim<N2, N1, N2> sim;
  private final DCMotor motor;

  /**
   * Constructs a new DumbMotorIOSim instance.
   *
   * @param motor The DC motor model to simulate
   * @param sim The linear system simulation representing the motor's behavior
   */
  public DumbMotorIOSim(DCMotor motor, LinearSystemSim<N2, N1, N2> sim) {
    this.motor = motor;
    this.sim = sim;
  }

  /**
   * Updates the inputs for the motor controller
   *
   * @param inputs The inputs object to update with current values
   */
  @Override
  public void updateInputs(DumbMotorIOInputs inputs) {
    sim.update(0.02);
    double inputVoltage = sim.getInput(0);
    double current = motor.getCurrent(sim.getOutput(0), inputVoltage);

    inputs.connected = true;
    inputs.appliedVolts = inputVoltage;
    inputs.currentAmps = current;
  }

  /**
   * Sets the motor to run at the specified voltage
   *
   * @param voltage The desired voltage to apply to the motor
   */
  @Override
  public void runVoltage(double voltage) {
    sim.setInput(voltage);
  }
}
