package org.team2342.frc.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import java.util.Queue;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import org.team2342.frc.Constants.DriveConstants;
import org.team2342.frc.util.PhoenixUtils;

/* ModuleIO implementation for dual TalonFX module with a CANcoder. */
public class ModuleIOTalonFX implements ModuleIO {
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;

  private final double offset;

  private final TalonFXConfiguration driveConfig;
  private final TalonFXConfiguration turnConfig;
  private static final Executor brakeModeExecutor = Executors.newFixedThreadPool(8);

  // Drive Inputs
  private final StatusSignal<Angle> drivePosition;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveCurrent;

  // Turn Inputs
  private final StatusSignal<Angle> turnAbsolutePosition;
  private final StatusSignal<Angle> turnPosition;
  private final StatusSignal<AngularVelocity> turnVelocity;
  private final StatusSignal<Voltage> turnAppliedVolts;
  private final StatusSignal<Current> turnCurrent;

  // High-frequency odometry queues
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage driveRequest = new VelocityVoltage(0);
  private final PositionVoltage turnRequest = new PositionVoltage(0);

  private final boolean driveInverted = true;
  private final boolean turnInverted = true;
  private final boolean encoderInverted = false;

  // For coupling ratio calculations
  // private double turnVel = 0.0;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnConnectedDebounce = new Debouncer(0.5);
  private final Debouncer encoderConnectedDebounce = new Debouncer(0.5);

  public ModuleIOTalonFX(int[] canIDArray, double encoderOffset) {
    driveTalon = new TalonFX(canIDArray[0]);
    turnTalon = new TalonFX(canIDArray[1]);
    cancoder = new CANcoder(canIDArray[2]);
    offset = encoderOffset;

    // Configure Drive
    driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Slot0 =
        new Slot0Configs().withKP(1.5).withKI(0.0).withKD(0.0).withKS(0.16).withKV(0.84);
    driveConfig.Feedback.SensorToMechanismRatio = DriveConstants.DRIVE_GEARING;
    driveConfig.MotorOutput.Inverted =
        driveInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    driveConfig.CurrentLimits.StatorCurrentLimit = DriveConstants.SLIP_CURRENT_LIMIT;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.CurrentLimits.SupplyCurrentLimit = DriveConstants.DRIVE_SUPPLY_LIMIT;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    PhoenixUtils.tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
    PhoenixUtils.tryUntilOk(5, () -> driveTalon.setPosition(0, 0.25));

    // Configure Turn
    turnConfig = new TalonFXConfiguration();
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnConfig.Slot0 =
        new Slot0Configs().withKP(100.0).withKI(0).withKD(0).withKS(0.154).withKV(2.556);
    turnConfig.CurrentLimits.SupplyCurrentLimit = DriveConstants.TURN_CURRENT_LIMIT;
    turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Set up CANcoder as feedback device for turn motor
    turnConfig.Feedback.FeedbackRemoteSensorID = canIDArray[2];
    turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    turnConfig.Feedback.RotorToSensorRatio = DriveConstants.TURN_GEARING;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.MotorOutput.Inverted =
        turnInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    PhoenixUtils.tryUntilOk(5, () -> turnTalon.getConfigurator().apply(turnConfig, 0.25));

    // Configure Encoder
    CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.MagnetOffset = offset;
    cancoderConfig.MagnetSensor.SensorDirection =
        encoderInverted
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    PhoenixUtils.tryUntilOk(5, () -> cancoder.getConfigurator().apply(cancoderConfig, 0.25));

    // Create input signals
    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();

    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getStatorCurrent();

    // Want position signals to run at higher frequency for odometry
    BaseStatusSignal.setUpdateFrequencyForAll(
        DriveConstants.ODOMETRY_FREQUENCY, drivePosition, turnPosition);
    timestampQueue = PhoenixOdometry.getInstance().makeTimestampQueue();
    drivePositionQueue = PhoenixOdometry.getInstance().registerSignal(drivePosition.clone());
    turnPositionQueue = PhoenixOdometry.getInstance().registerSignal(turnPosition.clone());

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);

    // Optimize utilization for everything
    PhoenixUtils.tryUntilOk(
        5, () -> ParentDevice.optimizeBusUtilizationForAll(driveTalon, turnTalon, cancoder));

    // Register signals for update
    PhoenixUtils.registerSignals(
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnPosition,
        turnAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.driveConnected =
        driveConnectedDebounce.calculate(
            BaseStatusSignal.isAllGood(
                drivePosition, driveVelocity, driveAppliedVolts, driveCurrent));
    inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

    inputs.turnConnected =
        turnConnectedDebounce.calculate(
            BaseStatusSignal.isAllGood(turnPosition, turnVelocity, turnAppliedVolts, turnCurrent));
    inputs.encoderConnected =
        encoderConnectedDebounce.calculate(BaseStatusSignal.isAllGood(turnAbsolutePosition));
    inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());

    // turnVel = turnVelocity.getValueAsDouble();
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());

    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = turnCurrent.getValueAsDouble();

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value))
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void runDriveVelocity(double velocityRadPerSec) {
    double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);

    driveTalon.setControl(
        driveRequest.withVelocity(
            velocityRotPerSec)); // + (-turnVel * DriveConstants.COUPLE_RATIO)));
  }

  @Override
  public void runDriveVoltage(double voltage) {
    driveTalon.setControl(voltageRequest.withOutput(voltage));
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnTalon.setControl(turnRequest.withPosition(rotation.getRotations()));
  }

  @Override
  public void runTurnVoltage(double voltage) {
    turnTalon.setControl(voltageRequest.withOutput(voltage));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    // Use executor to enable and disable brake mode
    brakeModeExecutor.execute(
        () -> {
          synchronized (driveConfig) {
            driveConfig.MotorOutput.NeutralMode =
                enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
            PhoenixUtils.tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
          }
        });
    brakeModeExecutor.execute(
        () -> {
          synchronized (turnConfig) {
            turnConfig.MotorOutput.NeutralMode =
                enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
            PhoenixUtils.tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
          }
        });
  }

  @Override
  public double getAbsoluteAngle() {
    return cancoder.getAbsolutePosition().getValueAsDouble() - offset;
  }
}
