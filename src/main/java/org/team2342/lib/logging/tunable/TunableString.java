// Copyright (c) 2025 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.logging.tunable;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

public class TunableString implements Supplier<String> {
  private static final String tableKey = "/Tuning";

  private final String key;
  private final BooleanSupplier tuningMode;
  private final String defaultValue;

  private LoggedNetworkString dashboardString;
  private Map<Integer, String> lastHasChangedValues = new HashMap<>();

  public TunableString(String dashboardKey, String defaultValue, BooleanSupplier tuningMode) {
    this.key = tableKey + "/" + dashboardKey;
    this.tuningMode = tuningMode;
    this.defaultValue = defaultValue;
    if (tuningMode.getAsBoolean()) {
      dashboardString = new LoggedNetworkString(key, defaultValue);
    }
  }

  @Override
  public String get() {
    return tuningMode.getAsBoolean() ? dashboardString.get() : defaultValue;
  }

  public boolean hasChanged(int hashcode) {
    String currentValue = get();
    String lastValue = lastHasChangedValues.get(hashcode);
    if (lastValue == null || currentValue != lastValue) {
      lastHasChangedValues.put(hashcode, currentValue);
      return true;
    }

    return false;
  }
}
