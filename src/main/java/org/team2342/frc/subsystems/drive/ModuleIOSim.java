package org.team2342.frc.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.team2342.frc.Constants.DriveConstants;

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
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1), 0.025, DriveConstants.DRIVE_GEARING),
            DCMotor.getKrakenX60(1));

    turnSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
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
    inputs.drivePositionRad = driveSim.getAngularPositionRad();
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

    inputs.turnConnected = true;
    inputs.encoderConnected = true;
    inputs.turnAbsolutePosition = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());

    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  @Override
  public void runDriveVelocity(double velocityRadPerSec) {
    driveAppliedVolts =
        driveController.calculate(driveSim.getAngularVelocityRadPerSec(), velocityRadPerSec)
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
        turnController.calculate(turnSim.getAngularPositionRad(), rotation.getRadians());
    turnSim.setInputVoltage(turnAppliedVolts);
  }

  @Override
  public void runTurnVoltage(double voltage) {
    turnAppliedVolts = voltage;
    turnSim.setInputVoltage(voltage);
  }
}
