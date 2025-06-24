package org.team2342.lib.sensors.distance;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/*
 * Implementing DistanceSensorIO for sim
 */

public class DistanceSensorIOSim implements DistanceSensorIO {

  private final String entryKey;

  public DistanceSensorIOSim(String entryKey) {
    this.entryKey = entryKey;
    // Set default value if it hasn't been set already
    if (!SmartDashboard.containsKey(entryKey)) {
      SmartDashboard.putNumber(entryKey, 0.5);
    }
  }

  /**
   * Gets called to update sensor reading
   * @param inputs The object that stores sensor data
   */
  @Override
  public void updateInputs(DistanceSensorIOInputs inputs) {
    // Get the current simulated distance from SmartDashboard
    inputs.distanceMeters = SmartDashboard.getNumber(entryKey, 0.5);
    inputs.connected = true;
  }
}
