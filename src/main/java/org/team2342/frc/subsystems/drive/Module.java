package org.team2342.frc.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.team2342.frc.Constants.DriveConstants;
import org.team2342.lib.logging.ExecutionLogger;

/** Class for controlling individual swerve modules */
public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

  private final int index;

  private final Alert driveAlert;
  private final Alert turnAlert;
  private final Alert encoderAlert;

  @Getter private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;

    // Create alerts for disconnected electronics
    driveAlert =
        new Alert(
            "Module " + Integer.toString(index) + " drive motor disconnected!", AlertType.kError);
    turnAlert =
        new Alert(
            "Module " + Integer.toString(index) + " turn motor disconnected!", AlertType.kError);
    encoderAlert =
        new Alert("Module " + Integer.toString(index) + " encoder disconnected!", AlertType.kError);
  }

  /** Periodic function for modules. */
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

    // Calculate positions for odometry
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsRad[i] * DriveConstants.WHEEL_RADIUS;
      Rotation2d angle = inputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    driveAlert.set(!inputs.driveConnected);
    turnAlert.set(!inputs.turnConnected);
    encoderAlert.set(!inputs.encoderConnected);

    // Record cycle time
    ExecutionLogger.log("Drive/Module" + index);
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  public void runSetpoint(SwerveModuleState state) {
    // Optimize states
    state.optimize(getAngle());
    state.cosineScale(getAngle());

    // Run the optimized setpoint
    io.runDriveVelocity(state.speedMetersPerSecond / DriveConstants.WHEEL_RADIUS);
    io.setTurnPosition(state.angle);
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(double output) {
    io.runDriveVoltage(output);
    io.setTurnPosition(new Rotation2d());
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.runDriveVoltage(0.0);
    io.runTurnVoltage(0.0);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * DriveConstants.WHEEL_RADIUS;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * DriveConstants.WHEEL_RADIUS;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRad;
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    return Units.radiansToRotations(inputs.driveVelocityRadPerSec);
  }

  /** Get value of absolute encoder without offset. */
  public double getAbsoluteAngle() {
    return io.getAbsoluteAngle();
  }
}
