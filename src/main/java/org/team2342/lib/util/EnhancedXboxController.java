// Copyright (c) 2025 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class EnhancedXboxController extends CommandXboxController {
  private double deadband;

  public EnhancedXboxController(int port, double deadband) {
    super(port);
  }

  public EnhancedXboxController(int port) {
    this(port, 0);
  }

  public Command rumble(RumbleType type, double intensity) {
    return Commands.startEnd(
        () -> super.setRumble(type, intensity), () -> super.setRumble(type, 0.0));
  }

  public Command timedRumble(RumbleType type, double intensity, double seconds) {
    return Commands.runEnd(() -> super.setRumble(type, intensity), () -> super.setRumble(type, 0.0))
        .withTimeout(seconds);
  }

  @Override
  public double getLeftX() {
    return MathUtil.applyDeadband(super.getLeftX(), deadband);
  }

  @Override
  public double getLeftY() {
    return MathUtil.applyDeadband(super.getLeftY(), deadband);
  }

  @Override
  public double getRightX() {
    return MathUtil.applyDeadband(super.getRightX(), deadband);
  }

  @Override
  public double getRightY() {
    return MathUtil.applyDeadband(super.getRightY(), deadband);
  }
}
