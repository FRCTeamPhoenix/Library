// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.visualizers;

import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.wpilib.util.Color8Bit;

public class ElevatorVisualizer extends LoggedMechanismLigament2d {

  private double startingHeight;

  public ElevatorVisualizer(String name, double startingHeight, double angleDeg, Color8Bit color) {
    super(name, startingHeight, angleDeg, 10, color);
    this.startingHeight = startingHeight;
  }

  public ElevatorVisualizer(String name, double startingHeight, Color8Bit color) {
    this(name, startingHeight, 90.0, color);
  }

  public void update(double height) {
    setLength(height + startingHeight);
  }
}
