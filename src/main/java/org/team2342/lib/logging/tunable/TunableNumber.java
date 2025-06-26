// Copyright (c) 2025 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.logging.tunable;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class TunableNumber implements DoubleSupplier {
  private static final String tableKey = "/Tuning";

  private final String key;
  private final BooleanSupplier tuningMode;
  private final double defaultValue;

  private LoggedNetworkNumber dashboardNumber;
  private Map<Integer, Double> lastHasChangedValues = new HashMap<>();

  public TunableNumber(String dashboardKey, double defaultValue, BooleanSupplier tuningMode) {
    this.key = tableKey + "/" + dashboardKey;
    this.tuningMode = tuningMode;
    this.defaultValue = defaultValue;
    if (tuningMode.getAsBoolean()) {
      dashboardNumber = new LoggedNetworkNumber(key, defaultValue);
    }
  }

  public double get() {
    return tuningMode.getAsBoolean() ? dashboardNumber.get() : defaultValue;
  }

  @Override
  public double getAsDouble() {
    return get();
  }

  public boolean hasChanged(int hashcode) {
    double currentValue = get();
    Double lastValue = lastHasChangedValues.get(hashcode);
    if (lastValue == null || currentValue != lastValue) {
      lastHasChangedValues.put(hashcode, currentValue);
      return true;
    }

    return false;
  }
}
