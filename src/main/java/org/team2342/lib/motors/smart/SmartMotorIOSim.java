package org.team2342.lib.motors.smart;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import org.team2342.lib.motors.smart.SmartMotorConfig.ControlType;
import org.team2342.lib.pidff.FeedforwardController;
import org.team2342.lib.pidff.PIDFFConfigs;

public class SmartMotorIOSim implements SmartMotorIO {

  private final SmartMotorConfig config;

  private final LinearSystemSim<N2, N1, N2> sim;

  private final PIDController pid;
  private final FeedforwardController ff;
  private final DCMotor motor;

  /**
   * Simulated Smart Motor class
   *
   * <p>Current limits won't be applied in simulation
   *
   * @param config The motor configuration
   * @param sim The WPILib simulation class representing your mechanism. Be sure to check whether
   *     you need to use the simRatio in the config or not.
   */
  public SmartMotorIOSim(SmartMotorConfig config, DCMotor motor, LinearSystemSim<N2, N1, N2> sim) {
    this.config = config;

    this.motor = motor;
    this.sim = sim;

    pid = new PIDController(config.pidffConfigs.kP, config.pidffConfigs.kI, config.pidffConfigs.kD);
    ff = new FeedforwardController(config.pidffConfigs);
  }

  @Override
  public void updateInputs(SmartMotorIOInputs inputs) {
    sim.update(0.02);

    inputs.motorsConnected = new boolean[] {true};
    inputs.positionRad = sim.getOutput(0) / config.simRatio;
    inputs.velocityRadPerSec = sim.getOutput(1) / config.simRatio;
    inputs.appliedVolts = new double[] {sim.getInput(0)};
    inputs.currentAmps =
        new double[] {
          motor.getCurrent(
              inputs.velocityRadPerSec * config.gearRatio / config.simRatio, inputs.appliedVolts[0])
        };
  }

  @Override
  public void runVelocity(double velocityRadPerSec) {
    if (config.controlType == ControlType.VELOCITY) {
      double currentVel = sim.getOutput(1);
      double control =
          pid.calculate(currentVel, velocityRadPerSec)
              + ff.calculate(velocityRadPerSec, 0, sim.getOutput(0));
      sim.setInput(control);
    } else {
      throw new IllegalStateException(
          "Cannot run velocity control: smart motor is configured for "
              + config.controlType.toString()
              + " control");
    }
  }

  @Override
  public void setPosition(double positionRad) {
    if (config.controlType == ControlType.POSITION) {
      double currentPos = sim.getOutput(0);
      double control =
          pid.calculate(currentPos, positionRad) + ff.calculate(0, 0, sim.getOutput(0));
      sim.setInput(control);
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
