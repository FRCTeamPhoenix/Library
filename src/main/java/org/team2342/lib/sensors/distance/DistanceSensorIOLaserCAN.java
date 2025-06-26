// Copyright (c) 2025 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.sensors.distance;

import au.grapplerobotics.LaserCan;

/*
 * Implementing DistanceSensorIO
 * Handling LaserCAN distance sensor
 */

public class DistanceSensorIOLaserCAN implements DistanceSensorIO {
  private final LaserCan laserCan;

  /**
   * Constructor to configure the LaserCAN
   *
   * @param canId The CAN id for the sensor
   * @param rangingMode The mode for distance measurement, SHORT or LONG range
   * @param timingBudget The time the sensor spends taking a measurement
   * @param regionOfInterest The part of the sensor's view to focus on for taking measurements
   */
  public DistanceSensorIOLaserCAN(
      int canId,
      LaserCan.RangingMode rangingMode,
      LaserCan.TimingBudget timingBudget,
      LaserCan.RegionOfInterest regionOfInterest) {
    this.laserCan = new LaserCan(canId);
    try {
      laserCan.setRangingMode(rangingMode);
      laserCan.setTimingBudget(timingBudget);
      laserCan.setRegionOfInterest(regionOfInterest);
    } catch (Exception e) {
      System.err.println("LaserCAN configuration failed: " + e.getMessage());
    }
  }

  /**
   * Gets called to update sensor reading
   *
   * @param inputs The object that stores sensor data
   */
  @Override
  public void updateInputs(DistanceSensorIOInputs inputs) {
    try {
      inputs.distanceMeters = laserCan.getMeasurement().distance_mm / 1000.0;
      inputs.connected = true;
    } catch (Exception e) {
      inputs.connected = false;
      inputs.distanceMeters = -1.0;
      System.err.println("Failed to read LaserCAN: " + e.getMessage());
    }
  }
}
