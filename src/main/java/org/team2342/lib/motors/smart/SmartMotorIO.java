// Copyright (c) 2025 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.motors.smart;

import org.littletonrobotics.junction.AutoLog;
import org.team2342.lib.pidff.PIDFFConfigs;

public interface SmartMotorIO {

  @AutoLog
  public class SmartMotorIOInputs {
    public boolean[] motorsConnected;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double[] appliedVolts = new double[] {0.0};
    public double[] currentAmps = new double[] {0.0};
  }

  public default void updateInputs(SmartMotorIOInputs inputs) {}

  public default void runPosition(double positionRad) {}

  public default void runVelocity(double velocityRadPerSec) {}

  public default void runVoltage(double voltage) {}

  public default void reconfigurePIDFF(PIDFFConfigs configs) {}

  public default void setPosition(double positionRad) {}
}
