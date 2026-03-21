// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.motors.dumb;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.Debouncer;
import org.team2342.lib.motors.MotorConfig;

public class DumbMotorIOSparkFlex implements DumbMotorIO {

  private final SparkFlex motor;
  private final SparkFlexConfig motorConfig = new SparkFlexConfig();
  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  /**
   * Constructor to configure the TalonFX motor controller
   *
   * @param canID The CAN ID of the TalonFX motor controller
   * @param config The configuration settings for the motor
   * @param type The type of motor being controlled
   */
  @SuppressWarnings("removal")
public DumbMotorIOSparkFlex(int canID, MotorConfig config, MotorType type) {
    motor = new SparkFlex(canID, type);
    motorConfig.inverted(config.motorInverted);
    motorConfig.idleMode(config.idleMode == MotorConfig.IdleMode.BRAKE
        ? SparkBaseConfig.IdleMode.kBrake
        : SparkBaseConfig.IdleMode.kCoast);
    motorConfig.smartCurrentLimit((int) config.supplyLimit);

    motor.configure(
        motorConfig,
        SparkFlex.ResetMode.kNoResetSafeParameters,
        SparkFlex.PersistMode.kNoPersistParameters);
  }

  /**
   * Updates the inputs for the motor controller
   *
   * @param inputs The inputs object to update with current values
   */
  @Override
  public void updateInputs(DumbMotorIOInputs inputs) {
    inputs.connected = connectedDebouncer.calculate(true);
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = motor.getOutputCurrent();
  }

  /**
   * Sets the motor to run at the specified voltage
   *
   * @param voltage The desired voltage to apply to the motor
   */
  @Override
  public void runVoltage(double volts) {
    motor.setVoltage(volts);
  }
}
