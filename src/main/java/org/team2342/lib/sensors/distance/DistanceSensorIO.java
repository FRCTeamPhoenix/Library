package org.team2342.lib.sensors.distance;

import org.littletonrobotics.junction.AutoLog;

public interface DistanceSensorIO {
  @AutoLog
  public static class DistanceSensorIOInputs {
    public boolean connected = false; 
    public double distanceMeters = 0.0;
  }

  public default void updateInputs(DistanceSensorIOInputs inputs) {}
}
