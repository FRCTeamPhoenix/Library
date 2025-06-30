package org.team2342.lib.motors.smart;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.team2342.frc.util.PhoenixUtils;
import org.team2342.lib.motors.smart.SmartMotorConfig.ControlType;
import org.team2342.lib.motors.smart.SmartMotorConfig.FollowerConfig;
import org.team2342.lib.motors.smart.SmartMotorConfig.IdleMode;
import org.team2342.lib.pidff.PIDFFConfigs;

public class SmartMotorIOTalonFX implements SmartMotorIO {

  private final TalonFX leaderTalon;
  private final TalonFX[] followerTalons;

  private final TalonFXConfiguration talonConfig;
  private final StatusSignal<Angle> leaderPosition;
  private final StatusSignal<AngularVelocity> leaderVelocity;
  private final StatusSignal<Voltage> leaderAppliedVolts;
  private final StatusSignal<Current> leaderCurrent;

  private final StatusSignal<Voltage>[] followersAppliedVolts;
  private final StatusSignal<Current>[] followersCurrent;

  private final VoltageOut voltageRequest = new VoltageOut(0);

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  private final PositionVoltage positionRequest = new PositionVoltage(0);

  private final MotionMagicVelocityVoltage velocityMotionMagicRequest =
      new MotionMagicVelocityVoltage(0);
  private final MotionMagicVoltage positionMotionMagicRequest = new MotionMagicVoltage(0);

  private final Debouncer leaderConnectedDebounce = new Debouncer(0.5);
  private final Debouncer[] followersConnectedDebounce;

  private final SmartMotorConfig config;

  @SuppressWarnings("unchecked") // lol type "safety"
  public SmartMotorIOTalonFX(int canID, SmartMotorConfig config, FollowerConfig... followers) {
    this.config = config;

    leaderTalon = new TalonFX(canID);

    // Configure Drive
    talonConfig = new TalonFXConfiguration();
    talonConfig.MotorOutput.NeutralMode =
        config.idleMode == IdleMode.BRAKE ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    talonConfig.Slot0 = Slot0Configs.from(config.pidffConfigs.asPhoenixSlotConfigs());
    talonConfig.Feedback.SensorToMechanismRatio = config.gearRatio;
    talonConfig.MotorOutput.Inverted =
        config.motorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    talonConfig.CurrentLimits.StatorCurrentLimit = config.statorLimit;
    talonConfig.CurrentLimits.StatorCurrentLimitEnable = config.statorLimit > 0;
    talonConfig.CurrentLimits.SupplyCurrentLimit = config.supplyLimit;
    talonConfig.CurrentLimits.SupplyCurrentLimitEnable = config.supplyLimit > 0;

    talonConfig.MotionMagic.MotionMagicCruiseVelocity =
        Units.radiansToRotations(config.profileConstraintsRad.maxVelocity);
    talonConfig.MotionMagic.MotionMagicAcceleration =
        Units.radiansToRotations(config.profileConstraintsRad.maxAcceleration);

    PhoenixUtils.tryUntilOk(5, () -> leaderTalon.getConfigurator().apply(talonConfig, 0.25));
    PhoenixUtils.tryUntilOk(5, () -> leaderTalon.setPosition(0, 0.25));

    // Create input signals
    leaderPosition = leaderTalon.getPosition();
    leaderVelocity = leaderTalon.getVelocity();
    leaderAppliedVolts = leaderTalon.getMotorVoltage();
    leaderCurrent = leaderTalon.getStatorCurrent();

    followerTalons = new TalonFX[followers.length];
    followersAppliedVolts = (StatusSignal<Voltage>[]) new StatusSignal[followers.length];
    followersCurrent = (StatusSignal<Current>[]) new StatusSignal[followers.length];
    followersConnectedDebounce = new Debouncer[followers.length];

    for (int i = 0; i < followers.length; i++) {
      Pair<Integer, Boolean> followerConfig = followers[i];
      followerTalons[i] = new TalonFX(followerConfig.getFirst());
      final int j = i;
      PhoenixUtils.tryUntilOk(
          5,
          () ->
              followerTalons[j].setControl(
                  new Follower(leaderTalon.getDeviceID(), followerConfig.getSecond())));
      followersAppliedVolts[i] = followerTalons[i].getMotorVoltage();
      followersCurrent[i] = followerTalons[i].getStatorCurrent();
      followersConnectedDebounce[i] = new Debouncer(0.5);
    }

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, followersAppliedVolts);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, followersCurrent);

    // Optimize utilization for everything
    PhoenixUtils.tryUntilOk(5, () -> ParentDevice.optimizeBusUtilizationForAll(leaderTalon));
    PhoenixUtils.tryUntilOk(5, () -> ParentDevice.optimizeBusUtilizationForAll(followerTalons));

    // Register signals for update
    PhoenixUtils.registerSignals(leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent);
    PhoenixUtils.registerSignals(followersAppliedVolts);
    PhoenixUtils.registerSignals(followersCurrent);
  }

  @Override
  public void updateInputs(SmartMotorIOInputs inputs) {
    inputs.motorsConnected = new boolean[1 + followerTalons.length];
    inputs.appliedVolts = new double[1 + followerTalons.length];
    inputs.currentAmps = new double[1 + followerTalons.length];

    inputs.motorsConnected[0] =
        leaderConnectedDebounce.calculate(
            BaseStatusSignal.isAllGood(
                leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent));
    inputs.positionRad = Units.rotationsToRadians(leaderPosition.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(leaderVelocity.getValueAsDouble());
    inputs.appliedVolts[0] = leaderAppliedVolts.getValueAsDouble();
    inputs.currentAmps[0] = leaderCurrent.getValueAsDouble();

    for (int i = 0; i < followerTalons.length; i++) {
      inputs.motorsConnected[i + 1] =
          followersConnectedDebounce[i].calculate(
              BaseStatusSignal.isAllGood(followersAppliedVolts[i], followersCurrent[i]));
      inputs.appliedVolts[i + 1] = followersAppliedVolts[i].getValueAsDouble();
      inputs.currentAmps[i + 1] = followersCurrent[i].getValueAsDouble();
    }
  }

  @Override
  public void runVelocity(double velocityRadPerSec) {
    if (config.controlType == ControlType.VELOCITY) {
      double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
      leaderTalon.setControl(velocityRequest.withVelocity(velocityRotPerSec));
    } else if (config.controlType == ControlType.PROFILED_VELOCITY) {
      double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
      leaderTalon.setControl(velocityMotionMagicRequest.withVelocity(velocityRotPerSec));
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
      leaderTalon.setControl(positionRequest.withPosition(positionRot));
    } else if (config.controlType == ControlType.PROFILED_POSITION) {
      double positionRot = Units.radiansToRotations(positionRad);
      leaderTalon.setControl(positionMotionMagicRequest.withPosition(positionRot));
    } else {
      throw new IllegalStateException(
          "Cannot run position control: smart motor is configured for "
              + config.controlType.toString()
              + " control");
    }
  }

  @Override
  public void runVoltage(double voltage) {
    leaderTalon.setControl(voltageRequest.withOutput(voltage));
  }

  @Override
  public void reconfigurePIDFF(PIDFFConfigs configs) {
    this.config.pidffConfigs = configs;
    talonConfig.Slot0 = Slot0Configs.from(config.pidffConfigs.asPhoenixSlotConfigs());
    PhoenixUtils.tryUntilOk(5, () -> leaderTalon.getConfigurator().apply(talonConfig, 0.25));
  }
}
