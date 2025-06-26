// Copyright (c) 2025 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.logging.tunable;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class TunableBoolean implements BooleanSupplier {
  private static final String tableKey = "/Tuning";

  private final String key;
  private final BooleanSupplier tuningMode;
  private final boolean defaultValue;

  private LoggedNetworkBoolean dashboardBoolean;
  private Map<Integer, Boolean> lastHasChangedValues = new HashMap<>();

  public TunableBoolean(String dashboardKey, boolean defaultValue, BooleanSupplier tuningMode) {
    this.key = tableKey + "/" + dashboardKey;
    this.tuningMode = tuningMode;
    this.defaultValue = defaultValue;
    if (tuningMode.getAsBoolean()) {
      dashboardBoolean = new LoggedNetworkBoolean(key, defaultValue);
    }
  }

  public boolean get() {
    return tuningMode.getAsBoolean() ? dashboardBoolean.get() : defaultValue;
  }

  @Override
  public boolean getAsBoolean() {
    return get();
  }

  public boolean hasChanged(int hashcode) {
    boolean currentValue = get();
    Boolean lastValue = lastHasChangedValues.get(hashcode);
    if (lastValue == null || currentValue != lastValue) {
      lastHasChangedValues.put(hashcode, currentValue);
      return true;
    }

    return false;
  }
}
