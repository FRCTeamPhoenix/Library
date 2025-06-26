// Copyright (c) 2025 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib;

import lombok.Getter;
import lombok.Setter;

@SuppressWarnings("unused")

// Class names use UpperCamelCase
public class Conventions {
  // Write constant names using SCREAMING_SNAKE_CASE
  // Constants are always public static final
  public static final double CONSTANT_NUMBER = 3.14159265;

  // Fields are lowerCamelCase
  // DO NOT use Hungarian notation when naming variables (m_someField)
  private double someField = 6.28318530;

  // Use get and set prefixes when creating methods to access variables
  // Method names are lowerCamelCase, and should describe what the method does
  public int getCurrentYear() {
    return 2025;
  }

  public void setSomeField(double someField) {
    // When setting fields, always use this.field for clarity
    this.someField = someField;
  }

  // You can also the @Setter and @Setter annotations to generate them
  @Getter @Setter private double anotherField = 10.0;

  // Units should always be written like the following:
  private double positionMeters = 1.0;
  private double velocityMetersPerSec = 10.0;
  private double accelerationMetersPerSecSq = 5.0;
  private double appliedVolts = 12.0;
  private double currentAmps = 40.0;
  // Use meters and radians, rather than inches, degrees, or something else

  // Enum names use UpperCamelCase
  // The values themselves are in SCREAMING_SNAKE_CASE
  public enum RgbColors {
    RED,
    GREEN,
    BLUE,
  }

  /**
   * When writing javadocs, put any relevant information about the inputs and outputs of a function
   * in. You don't need to explain the exact code of the function, just a general overview
   *
   * @param input Use &#064;param to specify inputs, and explain what your function wants
   * @return You can use &#064;return to explain what your function returns
   */
  public int exampleJavadocFunction(int input) {
    // Make sure your comments are appropriate and professional
    // After all, the code you write is public, and available for all to see
    int inputSquared = input * input;
    return inputSquared;
  }
}
