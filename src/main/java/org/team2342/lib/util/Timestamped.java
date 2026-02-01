// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.util;

import java.util.function.Supplier;
import lombok.Getter;

public class Timestamped<T> implements Supplier<T> {
  private T object;

  @Getter private double timestamp;

  public Timestamped(T object, double timestamp) {
    this.object = object;
    this.timestamp = timestamp;
  }

  @Override
  public T get() {
    return object;
  }
}
