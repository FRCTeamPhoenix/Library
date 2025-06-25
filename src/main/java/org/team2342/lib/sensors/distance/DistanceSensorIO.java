package org.team2342.lib.sensors.distance;

import org.littletonrobotics.junction.AutoLog;

/*
 * Interface for distance sensor input/output
 * Lets simulation, real hardware, and different sensor types to use same structure
 */
public interface DistanceSensorIO {
  /**
   * Container class for sensor inputs - used for logging and data updates @AutoLog automatically
   * generates code for logging with the AdvantageKit framework
   */
  @AutoLog
  public static class DistanceSensorIOInputs {
    public boolean connected = false;
    public double distanceMeters = 0.0;
  }

  /**
   * Called periodically to update the sensor input data
   *
   * @param inputs The object that stores sensor readings to be logged or used somewhere else
   */
  public default void updateInputs(DistanceSensorIOInputs inputs) {}
}
