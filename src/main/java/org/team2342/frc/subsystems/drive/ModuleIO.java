package org.team2342.frc.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public class ModuleIOInputs {
    public boolean driveConnected;
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;

    public boolean turnConnected;
    public boolean encoderConnected;
    public Rotation2d turnAbsolutePosition = new Rotation2d();
    public Rotation2d turnPosition = new Rotation2d();
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnCurrentAmps = 0.0;

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  /** Update loggable inputs object. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Run drive motor at specified velocity. */
  public default void runDriveVelocity(double velocityRadPerSec) {}

  /** Run drive motor at voltage. */
  public default void runDriveVoltage(double voltage) {}

  /** Run turn motor to specified position. */
  public default void setTurnPosition(Rotation2d rotation) {}

  /** Run turn motor at voltage. */
  public default void runTurnVoltage(double voltage) {}

  /** Set drive motor brake mode. */
  public default void setBrakeMode(boolean enabled) {}

  /** Get value of absolute encoder without offset. */
  public default double getAbsoluteAngle() {
    return 0.0;
  }
}
