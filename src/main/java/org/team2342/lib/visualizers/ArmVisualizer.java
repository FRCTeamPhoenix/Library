// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.visualizers;

import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.wpilib.util.Color8Bit;

public class ArmVisualizer extends LoggedMechanismLigament2d {

  public ArmVisualizer(String name, double length, double startingAngle, Color8Bit color) {
    super(name, length, startingAngle, 10, color);
  }

  public ArmVisualizer(String name, double length, Color8Bit color) {
    super(name, length, 0, 10, color);
  }

  public void update(double angle) {
    setAngle(angle);
  }
}
