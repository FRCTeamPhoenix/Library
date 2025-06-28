package org.team2342.lib.motors.smart;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import org.team2342.lib.motors.smart.SmartMotorConfig.ControlType;
import org.team2342.lib.pidff.FeedforwardController;
import org.team2342.lib.pidff.PIDFFConfigs;

public class SmartMotorIOSim implements SmartMotorIO {

  private final SmartMotorConfig config;

  private final LinearSystemSim<N2, N1, N2> sim;

  private final PIDController pid;
  private final ProfiledPIDController profiled;
  private final FeedforwardController ff;
  private final DCMotor motor;

  private final int followers;

  /**
   * Simulated Smart Motor class
   *
   * <p>Current limits won't be applied in simulation
   *
   * <p>Profiled Velocity is not currently functional, the sim will default to just velocity control
   *
   * @param config The motor configuration
   * @param sim The WPILib simulation class representing your mechanism. Be sure to check whether
   *     you need to use the simRatio in the config or not.
   * @param followers How many followers the normal motor class has, so the inputs object can be
   *     correctly sized
   */
  public SmartMotorIOSim(
      SmartMotorConfig config, DCMotor motor, LinearSystemSim<N2, N1, N2> sim, int followers) {
    this.config = config;

    this.motor = motor;
    this.sim = sim;

    this.followers = followers;

    if (config.controlType == ControlType.PROFILED_POSITION) {
      pid = null;
      profiled =
          new ProfiledPIDController(
              config.pidffConfigs.kP,
              config.pidffConfigs.kI,
              config.pidffConfigs.kD,
              new Constraints(
                  Units.radiansToRotations(config.profileConstraintsRad.maxVelocity),
                  Units.radiansToRotations(config.profileConstraintsRad.maxAcceleration)));
    } else {
      pid =
          new PIDController(config.pidffConfigs.kP, config.pidffConfigs.kI, config.pidffConfigs.kD);
      profiled = null;
    }
    ff = new FeedforwardController(config.pidffConfigs);
  }

  @Override
  public void updateInputs(SmartMotorIOInputs inputs) {
    sim.update(0.02);

    inputs.motorsConnected = new boolean[followers];
    inputs.appliedVolts = new double[followers];
    inputs.currentAmps = new double[followers];

    inputs.positionRad = sim.getOutput(0) / config.simRatio;
    inputs.velocityRadPerSec = sim.getOutput(1) / config.simRatio;

    for (int i = 0; i < followers + 1; i++) {
      inputs.motorsConnected[i] = true;
      inputs.appliedVolts[i] = sim.getInput(0);
      inputs.appliedVolts[i] =
          motor.getCurrent(
              inputs.velocityRadPerSec * config.gearRatio / config.simRatio,
              inputs.appliedVolts[0]);
    }
  }

  @Override
  public void runVelocity(double velocityRadPerSec) {
    if (config.controlType == ControlType.VELOCITY
        || config.controlType == ControlType.PROFILED_VELOCITY) {
      double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
      double currentVel = sim.getOutput(1) / config.simRatio;
      double control =
          pid.calculate(currentVel, velocityRotPerSec)
              + ff.calculate(velocityRotPerSec, 0, sim.getOutput(0) / config.simRatio);
      sim.setInput(control);
    } else {
      throw new IllegalStateException(
          "Cannot run velocity control: smart motor is configured for "
              + config.controlType.toString()
              + " control");
    }
  }

  @Override
  public void runPosition(double positionRad) {
    if (config.controlType == ControlType.POSITION) {
      double positionRot = Units.radiansToRotations(positionRad);
      double currentPos = sim.getOutput(0) / config.simRatio;
      double control =
          pid.calculate(currentPos, positionRot)
              + ff.calculate(0, 0, sim.getOutput(0) / config.simRatio);
      sim.setInput(control);
    } else if (config.controlType == ControlType.PROFILED_POSITION) {
      double positionRot = Units.radiansToRotations(positionRad);
      double currentPos = sim.getOutput(0) / config.simRatio;
      double pidOutput = profiled.calculate(currentPos, positionRot);
      double ffOutput =
          ff.calculate(profiled.getSetpoint().velocity, 0, sim.getOutput(0) / config.simRatio);
      sim.setInput(pidOutput + ffOutput);
    } else {
      throw new IllegalStateException(
          "Cannot run velocity control: smart motor is configured for "
              + config.controlType.toString()
              + " control");
    }
  }

  @Override
  public void runVoltage(double voltage) {
    sim.setInput(voltage);
  }

  @Override
  public void reconfigurePIDFF(PIDFFConfigs configs) {
    this.config.pidffConfigs = configs;
    pid.setPID(configs.kP, configs.kI, configs.kD);
    ;
    ff.setConfigs(configs);
  }
}
