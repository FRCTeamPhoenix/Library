// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.visualizers;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class VisualizerRoot {
  private final String name;
  private final LoggedMechanism2d mech;
  private final LoggedMechanismRoot2d root;

  public VisualizerRoot(String name, double baseWidth, double rootX, double rootY) {
    this.name = name;
    mech = new LoggedMechanism2d(baseWidth, baseWidth);
    root = mech.getRoot("root", rootX, rootY);
  }

  public void append(LoggedMechanismLigament2d ligament) {
    root.append(ligament);
  }

  public void update() {
    Logger.recordOutput("Mechanism/2D" + name, mech);
  }
}
