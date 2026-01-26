// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.motors.smart;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.team2342.frc.util.PhoenixUtils;
import org.team2342.lib.motors.MotorConfig;
import org.team2342.lib.motors.smart.SmartMotorConfig.ControlType;
import org.team2342.lib.motors.smart.SmartMotorConfig.FeedbackConfig;
import org.team2342.lib.motors.smart.SmartMotorConfig.FollowerConfig;
import org.team2342.lib.pidff.PIDFFConfigs;

public class SmartMotorIOTalonFXFOC implements SmartMotorIO {

  private final TalonFX leaderTalon;
  private final TalonFX[] followerTalons;

  private final TalonFXConfiguration talonConfig;
  private final StatusSignal<Angle> leaderPosition;
  private final StatusSignal<AngularVelocity> leaderVelocity;
  private final StatusSignal<Voltage> leaderAppliedVolts;
  private final StatusSignal<Current> leaderCurrent;

  private CANcoder cancoder = null;
  private StatusSignal<Angle> absolutePosition;

  private final StatusSignal<Voltage>[] followersAppliedVolts;
  private final StatusSignal<Current>[] followersCurrent;

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);

  private final VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0);
  private final PositionTorqueCurrentFOC positionRequest = new PositionTorqueCurrentFOC(0);

  private final MotionMagicVelocityTorqueCurrentFOC velocityMotionMagicRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0);
  private final MotionMagicTorqueCurrentFOC positionMotionMagicRequest =
      new MotionMagicTorqueCurrentFOC(0);

  private final Debouncer leaderConnectedDebounce = new Debouncer(0.5);
  private final Debouncer[] followersConnectedDebounce;

  private final SmartMotorConfig config;

  @SuppressWarnings("unchecked") // lol type "safety"
  public SmartMotorIOTalonFXFOC(int canID, SmartMotorConfig config, FollowerConfig... followers) {
    this.config = config;

    leaderTalon = new TalonFX(canID);

    talonConfig = new TalonFXConfiguration();
    configureTalon();
    configureFeedback();
    if (cancoder != null) {
      absolutePosition = cancoder.getAbsolutePosition();
      BaseStatusSignal.setUpdateFrequencyForAll(50.0, absolutePosition);
      PhoenixUtils.registerSignals(absolutePosition);
      PhoenixUtils.tryUntilOk(5, () -> ParentDevice.optimizeBusUtilizationForAll(cancoder));
    }

    PhoenixUtils.tryUntilOk(5, () -> leaderTalon.getConfigurator().apply(talonConfig, 0.25));
    PhoenixUtils.tryUntilOk(5, () -> leaderTalon.setPosition(0, 0.25));

    // Create input signals
    leaderPosition = leaderTalon.getPosition();
    leaderVelocity = leaderTalon.getVelocity();
    leaderAppliedVolts = leaderTalon.getMotorVoltage();
    leaderCurrent = leaderTalon.getTorqueCurrent();

    followerTalons = new TalonFX[followers.length];
    followersAppliedVolts = (StatusSignal<Voltage>[]) new StatusSignal[followers.length];
    followersCurrent = (StatusSignal<Current>[]) new StatusSignal[followers.length];
    followersConnectedDebounce = new Debouncer[followers.length];

    for (int i = 0; i < followers.length; i++) {
      FollowerConfig followerConfig = followers[i];
      followerTalons[i] = new TalonFX(followerConfig.canID());
      final int j = i;

      var followerConfigFX = new TalonFXConfiguration();
      followerConfigFX.MotorOutput.NeutralMode =
          config.idleMode == MotorConfig.IdleMode.BRAKE
              ? NeutralModeValue.Brake
              : NeutralModeValue.Coast;

      PhoenixUtils.tryUntilOk(
          5, () -> followerTalons[j].getConfigurator().apply(followerConfigFX, 0.25));

      PhoenixUtils.tryUntilOk(
          5,
          () ->
              followerTalons[j].setControl(
                  new Follower(
                      leaderTalon.getDeviceID(),
                      followerConfig.inverted()
                          ? MotorAlignmentValue.Opposed
                          : MotorAlignmentValue.Aligned)));
      followersAppliedVolts[i] = followerTalons[i].getMotorVoltage();
      followersCurrent[i] = followerTalons[i].getTorqueCurrent();
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

    if (cancoder != null) {
      inputs.absEncoderPositionRad = Units.rotationsToRadians(absolutePosition.getValueAsDouble());
    }

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
  public void runTorqueCurrent(double amps) {
    leaderTalon.setControl(torqueCurrentRequest.withOutput(amps));
  }

  @Override
  public void reconfigurePIDFF(PIDFFConfigs configs) {
    this.config.pidffConfigs = configs;
    talonConfig.Slot0 = Slot0Configs.from(config.pidffConfigs.asPhoenixSlotConfigs());
    PhoenixUtils.tryUntilOk(5, () -> leaderTalon.getConfigurator().apply(talonConfig, 0.25));
  }

  @Override
  public void setPosition(double positionRad) {
    PhoenixUtils.tryUntilOk(
        5, () -> leaderTalon.setPosition(Units.radiansToRotations(positionRad), 0.25));
  }

  private void configureTalon() {
    talonConfig.MotorOutput.NeutralMode =
        config.idleMode == MotorConfig.IdleMode.BRAKE
            ? NeutralModeValue.Brake
            : NeutralModeValue.Coast;
    talonConfig.Slot0 = Slot0Configs.from(config.pidffConfigs.asPhoenixSlotConfigs());
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
  }

  private void configureFeedback() {
    switch (config.feedbackConfig.type()) {
      case INTERNAL -> {
        talonConfig.Feedback.SensorToMechanismRatio = config.gearRatio;
      }

      case REMOTE -> {
        configureCANcoder(config.feedbackConfig);

        talonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        talonConfig.Feedback.FeedbackRemoteSensorID = config.feedbackConfig.encoderID();
        talonConfig.Feedback.SensorToMechanismRatio = config.gearRatio;
      }

      case FUSED -> {
        configureCANcoder(config.feedbackConfig);

        talonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        talonConfig.Feedback.FeedbackRemoteSensorID = config.feedbackConfig.encoderID();
        talonConfig.Feedback.RotorToSensorRatio = config.gearRatio;
        talonConfig.Feedback.SensorToMechanismRatio = 1.0;
      }
    }
  }

  private void configureCANcoder(FeedbackConfig feedback) {
    cancoder = new CANcoder(feedback.encoderID());

    var cfg = new CANcoderConfiguration();
    cfg.MagnetSensor.SensorDirection =
        feedback.inverted()
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    cfg.MagnetSensor.MagnetOffset = feedback.offsetRotations();

    PhoenixUtils.tryUntilOk(5, () -> cancoder.getConfigurator().apply(cfg, 0.25));
  }
}
