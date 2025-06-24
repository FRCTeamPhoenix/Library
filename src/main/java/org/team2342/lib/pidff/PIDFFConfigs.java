package org.team2342.lib.pidff;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.revrobotics.spark.config.ClosedLoopConfig;

/**
 * Class to store PID and FF configuration constants.
 *
 * <p>Method chaining is used for readability.
 */
public class PIDFFConfigs {

  // PID Constants
  public double kP = 0;
  public double kI = 0;
  public double kD = 0;

  // FF Constants
  public double kS = 0;
  public double kV = 0;
  public double kA = 0;

  // kG
  public double kG = 0;
  public GravityType gravityType = GravityType.STATIC;

  public PIDFFConfigs() {}

  /**
   * Modifies the kP parameter
   *
   * @param kP New proportional gain
   * @return Itself
   */
  public PIDFFConfigs withKP(double kP) {
    this.kP = kP;
    return this;
  }

  /**
   * Modifies the kI parameter
   *
   * @param kI New integral gain
   * @return Itself
   */
  public PIDFFConfigs withKI(double kI) {
    this.kI = kI;
    return this;
  }

  /**
   * Modifies the kD parameter
   *
   * @param kD New derivative gain
   * @return Itself
   */
  public PIDFFConfigs withKD(double kD) {
    this.kD = kD;
    return this;
  }

  /**
   * Modifies the kS parameter
   *
   * @param kS New static feedforward gain
   * @return Itself
   */
  public PIDFFConfigs withKS(double kS) {
    this.kS = kS;
    return this;
  }

  /**
   * Modifies the kV parameter
   *
   * @param kV New velocity feedforward gain
   * @return Itself
   */
  public PIDFFConfigs withKV(double kV) {
    this.kV = kV;
    return this;
  }

  /**
   * Modifies the kA parameter
   *
   * @param kA New acceleration feedforward gain
   * @return Itself
   */
  public PIDFFConfigs withKA(double kA) {
    this.kA = kA;
    return this;
  }

  /**
   * Modifies the kG parameter
   *
   * @param kG New gravity feedforward gain
   * @return Itself
   */
  public PIDFFConfigs withKG(double kG) {
    this.kG = kG;
    return this;
  }

  /**
   * Modifies the gravity type parameter
   *
   * @param gravityType New gravity feedforward type
   * @return Itself
   */
  public PIDFFConfigs withGravityType(GravityType gravityType) {
    this.gravityType = gravityType;
    return this;
  }

  /**
   * Converts all of the PIDFF configs to a {@link com.ctre.phoenix6.configs.SlotConfigs} for use
   * with CTRE motors
   *
   * @return The converted SlotConfigs
   */
  public SlotConfigs asPhoenixSlotConfigs() {
    return new SlotConfigs()
        .withKP(kP)
        .withKI(kI)
        .withKD(kD)
        .withKS(kS)
        .withKV(kV)
        .withKA(kA)
        .withKG(kG)
        .withGravityType(
            gravityType == GravityType.STATIC
                ? GravityTypeValue.Elevator_Static
                : GravityTypeValue.Arm_Cosine);
  }

  /**
   * Converts the PID configs to a {@link com.revrobotics.spark.config.ClosedLoopConfig} for use
   * with REV motors
   *
   * <p>NOTE: REV's ClosedLoopConfig doesn't support physics-based feedforwards, so the kS, kV, kA
   * and kG terms need to be applied separately.
   *
   * @return The converted ClosedLoopConfig
   */
  public ClosedLoopConfig asREVLibClosedLoopConfig() {
    return new ClosedLoopConfig().pid(kP, kI, kD);
  }

  /**
   * Compares another PIDFFConfigs with this object
   *
   * @param other Other object
   * @return True if the configs are the same
   */
  public boolean equals(PIDFFConfigs other) {
    return this.kP == other.kP
        && this.kI == other.kI
        && this.kD == other.kD
        && this.kS == other.kS
        && this.kV == other.kV
        && this.kA == other.kA
        && this.kG == other.kG
        && this.gravityType == other.gravityType;
  }

  /** Determines the type of gravity feedforward */
  public enum GravityType {
    /** Constant gravity feedforward, such as in an elevator */
    STATIC,

    /** Gravity feedforward that changes with the rotation of the mechanism, such as in an arm */
    COSINE
  }
}
