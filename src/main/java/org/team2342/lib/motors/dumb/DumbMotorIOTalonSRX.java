// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.motors.dumb;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.MathUtil;
import org.team2342.lib.motors.MotorConfig;

public class DumbMotorIOTalonSRX implements DumbMotorIO {
  private final TalonSRX talon;

  public DumbMotorIOTalonSRX(int canID, MotorConfig config) {
    talon = new TalonSRX(canID);

    talon.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(config.supplyLimit > 0, config.supplyLimit, 0, 0.1));

    talon.setInverted(config.motorInverted ? true : false);

    talon.setNeutralMode(
        config.idleMode == MotorConfig.IdleMode.BRAKE ? NeutralMode.Brake : NeutralMode.Coast);
  }

  @Override
  public void updateInputs(DumbMotorIOInputs inputs) {
    inputs.connected = true;
    inputs.appliedVolts = talon.getMotorOutputVoltage();
    inputs.currentAmps = talon.getSupplyCurrent();
  }

  @Override
  public void runVoltage(double voltage) {
    talon.set(ControlMode.PercentOutput, MathUtil.clamp(voltage / 12, -1, 1));
    ;
  }
}
