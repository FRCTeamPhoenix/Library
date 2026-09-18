// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc.subsystems.drive;

import org.team2342.frc.Constants.DriveConstants;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.math.util.Units;
import org.wpilib.simulation.DCMotorSim;
import org.wpilib.system.Timer;

/* ModuleIO implementation using WPILib simulation classes. */
public class ModuleIOSim implements ModuleIO {
  private final DCMotorSim driveSim;
  private final DCMotorSim turnSim;

  private PIDController driveController = new PIDController(0.4, 0.0, 0.000001);
  private PIDController turnController = new PIDController(10.0, 0.0, 0.0);

  // Need to convert units for feedforward from rotations to radians
  private double kvRot = 0.80512;
  private double kvRad = 1.0 / Units.rotationsToRadians(1.0 / kvRot);
  private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.006, kvRad);

  private double driveAppliedVolts = 1.0;
  private double turnAppliedVolts = 1.0;

  public ModuleIOSim() {
    // Create DCMotorSims for each motor
    driveSim =
        new DCMotorSim(
            Models.singleJointedArmFromPhysicalConstants(
                DCMotor.getKrakenX60(1), 0.025, DriveConstants.DRIVE_GEARING),
            DCMotor.getKrakenX60(1));

    turnSim =
        new DCMotorSim(
            Models.singleJointedArmFromPhysicalConstants(
                DCMotor.getKrakenX60(1), 0.004, DriveConstants.TURN_GEARING),
            DCMotor.getKrakenX60(1));

    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Need to run update, otherwise the simulation won't change
    driveSim.update(0.02);
    turnSim.update(0.02);

    inputs.driveConnected = true;
    inputs.drivePositionRad = driveSim.getAngularPosition();
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocity();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDraw());

    inputs.turnConnected = true;
    inputs.encoderConnected = true;
    inputs.turnAbsolutePosition = new Rotation2d(turnSim.getAngularPosition());
    inputs.turnPosition = new Rotation2d(turnSim.getAngularPosition());
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocity();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDraw());

    inputs.odometryTimestamps = new double[] {Timer.getTimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  @Override
  public void runDriveVelocity(double velocityRadPerSec) {
    driveAppliedVolts =
        driveController.calculate(driveSim.getAngularVelocity(), velocityRadPerSec)
            + ff.calculate(velocityRadPerSec);
    driveSim.setInputVoltage(driveAppliedVolts);
  }

  @Override
  public void runDriveVoltage(double voltage) {
    driveAppliedVolts = voltage;
    driveSim.setInputVoltage(voltage);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnAppliedVolts =
        turnController.calculate(turnSim.getAngularPosition(), rotation.getRadians());
    turnSim.setInputVoltage(turnAppliedVolts);
  }

  @Override
  public void runTurnVoltage(double voltage) {
    turnAppliedVolts = voltage;
    turnSim.setInputVoltage(voltage);
  }
}
