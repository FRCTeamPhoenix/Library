// Copyright (c) 2025 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.leds;

import com.ctre.phoenix.led.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import java.util.EnumSet;

/**
 * High-level LED control that splits a strip into two halves: one for the driver and one for the
 * operator. Each side can be assigned a color and an effect (solid, flashing, animations).
 *
 * <p>Use `setDriver()` and `setOperator()` to configure each side. Call `periodic()` each loop to
 * update animations or effects. Example usage in robot code:
 *
 * <pre>{@code
 * ledStrip.setDriver(Color.kRed, LedStrip.Effect.FIRE);
 * ledStrip.setOperator(Color.kBlue, LedStrip.Effect.FLASHING);
 * ledStrip.periodic(); // must be called regularly
 * }</pre>
 */
public class LedStrip {
  // LED options to choose from
  public enum Effect {
    SOLID,
    FLASHING,
    RAINBOW,
    FIRE,
    LARSON,
    OFF
  }

  private final LedIO io;
  private final int totalLeds;

  private Color driverColor = Color.kWhite;
  private Effect driverEffect = Effect.SOLID;

  private Color operatorColor = Color.kWhite;
  private Effect operatorEffect = Effect.SOLID;

  private boolean flashing = true;
  private double lastFlashTime = 0.0;

  // Set of effects that use animations
  private static final EnumSet<Effect> animations =
      EnumSet.of(Effect.RAINBOW, Effect.FIRE, Effect.LARSON);

  /**
   * Constructor for LedStrip.
   *
   * @param io The LedIO interface to control the LEDs.
   * @param totalLeds The total number of LEDs in the strip.
   */
  public LedStrip(LedIO io, int totalLeds) {
    this.io = io;
    this.totalLeds = totalLeds;
  }

  /**
   * Sets the color and effect for the driver side of the LED strip.
   *
   * @param color The color to set for the driver side.
   * @param effect The effect to apply to the driver side.
   */
  public void setDriverColor(Color color, Effect effect) {
    driverColor = color;
    driverEffect = effect;
  }

  // Same as above, but for the operator side
  public void setOperatorColor(Color color, Effect effect) {
    operatorColor = color;
    operatorEffect = effect;
  }

  /**
   * Updates the LED strip each cycle. Must be called regularly to keep flashing and animation
   * effects in sync.
   */
  public void periodic() {
    double now = Timer.getFPGATimestamp();

    if ((now - lastFlashTime) > 0.5) {
      flashing = !flashing;
      lastFlashTime = now;
    }

    // Driver
    if (animations.contains(driverEffect)) {
      io.setAnimation(createAnimation(driverColor, driverEffect));
    } else {
      io.setFirstHalfColor(resolve(driverColor, driverEffect));
    }

    // Operator
    if (animations.contains(operatorEffect)) {
      io.setAnimation(createAnimation(operatorColor, operatorEffect));
    } else {
      io.setSecondHalfColor(resolve(operatorColor, operatorEffect));
    }
  }

  /**
   * Converts an effect and color into a real color value Used for SOLID, FLASHING, and OFF effects.
   *
   * @param color The base color to resolve
   * @param effect The effect to apply
   * @return The resolved color based on the effect.
   */
  private Color resolve(Color color, Effect effect) {
    return switch (effect) {
      case SOLID -> color;
      case FLASHING -> flashing ? color : Color.kBlack;
      case OFF -> Color.kBlack;
      default -> color;
    };
  }

  /**
   * Builds an animation object (CTRE) from the given effect. Used for RAINBOW, FIRE, and LARSON
   * effects.
   *
   * @param color The base color for the animation.
   * @param effect The effect type to create.
   * @return The created Animation object.
   */
  private Animation createAnimation(Color color, Effect effect) {
    return switch (effect) {
      case RAINBOW -> new RainbowAnimation(1.0, 0.5, totalLeds, false, 0);
      case FIRE -> new FireAnimation(1.0, 0.5, totalLeds, 0.6, 0.1);
      case LARSON -> new LarsonAnimation(
          (int) (color.red * 255),
          (int) (color.green * 255),
          (int) (color.blue * 255),
          0,
          0.5,
          totalLeds,
          LarsonAnimation.BounceMode.Front,
          (totalLeds / 5));
      default -> null;
    };
  }
}
