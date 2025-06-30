// Copyright (c) 2025 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.motors.dumb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.team2342.frc.util.PhoenixUtils;

public class DumbMotorIOTalonFX implements DumbMotorIO {
  private final TalonFX talon;

  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  public DumbMotorIOTalonFX(int canID, DumbMotorConfig config) {
    talon = new TalonFX(canID);

    TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    talonConfig.CurrentLimits.SupplyCurrentLimit = config.supplyLimit;
    talonConfig.CurrentLimits.SupplyCurrentLimitEnable = config.supplyLimit > 0;
    talonConfig.CurrentLimits.StatorCurrentLimit = config.statorLimit;
    talonConfig.CurrentLimits.StatorCurrentLimitEnable = config.statorLimit > 0;

    talonConfig.MotorOutput.Inverted =
        config.motorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    talonConfig.MotorOutput.NeutralMode =
        config.idleMode == DumbMotorConfig.IdleMode.BRAKE
            ? NeutralModeValue.Brake
            : NeutralModeValue.Coast;

    PhoenixUtils.tryUntilOk(5, () -> talon.getConfigurator().apply(talonConfig, 0.5));

    appliedVolts = talon.getMotorVoltage();
    current = talon.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, appliedVolts, current);
    PhoenixUtils.registerSignals(appliedVolts, current);
    PhoenixUtils.tryUntilOk(5, () -> ParentDevice.optimizeBusUtilizationForAll(talon));
  }

  @Override
  public void updateInputs(DumbMotorIOInputs inputs) {
    inputs.connected =
        connectedDebouncer.calculate(BaseStatusSignal.isAllGood(appliedVolts, current));
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
  }

  @Override
  public void runVoltage(double voltage) {
    talon.setControl(voltageRequest.withOutput(voltage));
  }
}
