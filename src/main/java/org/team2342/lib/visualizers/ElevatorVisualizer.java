// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.visualizers;

import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.wpilib.util.Color8Bit;

public class ElevatorVisualizer extends LoggedMechanismLigament2d {

  public ElevatorVisualizer(String name, double startingHeight, double angle, Color8Bit color) {
    super(name, startingHeight, angle, 10, color);
  }

  public ElevatorVisualizer(String name, double startingHeight, Color8Bit color) {
    this(name, startingHeight, 90.0, color);
  }

  public void update(double height) {
    setLength(height);
  }
}
