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
import lombok.Getter;
import lombok.Setter;

/** Extended {@link CommandXboxController} class */
public class EnhancedXboxController extends CommandXboxController {

  @Getter @Setter private double deadband;

  /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into.
   * @param deadband The deadband that will be applied to the controller sticks.
   */
  public EnhancedXboxController(int port, double deadband) {
    super(port);
  }

  /**
   * Construct an instance of a controller with no deadband.
   *
   * @param port The port index on the Driver Station that the controller is plugged into.
   */
  public EnhancedXboxController(int port) {
    this(port, 0);
  }

  /**
   * Set the rumble output for the controller.
   *
   * @param type Which rumble value to set
   * @param intensity The normalized value (0 to 1) to set the rumble to
   * @return A command to rumble the controller
   */
  public Command rumble(RumbleType type, double intensity) {
    return Commands.startEnd(
        () -> super.setRumble(type, intensity), () -> super.setRumble(type, 0.0));
  }

  /**
   * Set the rumble output for the controller with a specified timeout.
   *
   * @param type Which rumble value to set
   * @param intensity The normalized value (0 to 1) to set the rumble to
   * @param seconds The length of time to rumble the controller for in seconds
   * @return A command to rumble the controller for the specifed time
   */
  public Command timedRumble(RumbleType type, double intensity, double seconds) {
    return Commands.runEnd(() -> super.setRumble(type, intensity), () -> super.setRumble(type, 0.0))
        .withTimeout(seconds);
  }

  /** Get the deadbanded X axis value of left side of the controller. Right is positive. */
  @Override
  public double getLeftX() {
    return MathUtil.applyDeadband(super.getLeftX(), deadband);
  }

  /** Get the deadbanded Y axis value of left side of the controller. Back is positive. */
  @Override
  public double getLeftY() {
    return MathUtil.applyDeadband(super.getLeftY(), deadband);
  }

  /** Get the deadbanded X axis value of right side of the controller. Right is positive. */
  @Override
  public double getRightX() {
    return MathUtil.applyDeadband(super.getRightX(), deadband);
  }

  /** Get the deadbanded Y axis value of right side of the controller. Back is positive. */
  @Override
  public double getRightY() {
    return MathUtil.applyDeadband(super.getRightY(), deadband);
  }
}
