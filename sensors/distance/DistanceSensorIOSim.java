// Copyright (c) 2025 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.sensors.distance;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
 * Implementing DistanceSensorIO for sim
 */

public class DistanceSensorIOSim implements DistanceSensorIO {

  private final String entryKey;
  private final double defaultValue;
  /**
   * Constructor to configure the sim
   *
   * @param entryKey Unique key to identify this entry
   * @param defaultValue Just like the name suggests
   */
  public DistanceSensorIOSim(String entryKey, double defaultValue) {
    this.entryKey = entryKey;
    this.defaultValue = defaultValue;
    // Set default value if it hasn't been set already
    if (!SmartDashboard.containsKey(entryKey)) {
      SmartDashboard.putNumber(entryKey, defaultValue);
    }
  }

  /**
   * Gets called to update sensor reading
   *
   * @param inputs The object that stores sensor data
   */
  @Override
  public void updateInputs(DistanceSensorIOInputs inputs) {
    // Get the current simulated distance from SmartDashboard
    inputs.distanceMeters = SmartDashboard.getNumber(entryKey, defaultValue);
    inputs.connected = true;
  }
}
