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

public class DumbMotorIOSim implements DumbMotorIO {
  private final LinearSystemSim<N2, N1, N2> sim;
  private final DCMotor motor;

  public DumbMotorIOSim(DCMotor motor, LinearSystemSim<N2, N1, N2> sim) {
    this.motor = motor;
    this.sim = sim;
  }

  @Override
  public void updateInputs(DumbMotorIOInputs inputs) {
    sim.update(0.02);
    double inputVoltage = sim.getInput(0);
    double current = motor.getCurrent(sim.getOutput(0), inputVoltage);

    inputs.connected = true;
    inputs.appliedVolts = inputVoltage;
    inputs.currentAmps = current;
  }

  @Override
  public void runVoltage(double voltage) {
    sim.setInput(voltage);
  }
}
