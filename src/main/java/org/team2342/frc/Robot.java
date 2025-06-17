package org.team2342.frc;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathShared;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.team2342.frc.util.PhoenixUtils;
import org.team2342.lib.fsm.StateMachine;
import org.team2342.lib.logging.ExecutionLogger;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private static final double loopOverrunWarningTimeout = 0.2;

  private final RobotContainer robotContainer;

  public enum testStates {
    UNDETERMINED,
    STATE_1,
    STATE_2,
    STATE_3,
    STATE_4
  }

  private StateMachine<testStates> machine =
      new StateMachine<Robot.testStates>(
          "TestMachine", testStates.UNDETERMINED, () -> testStates.STATE_1, testStates.class);

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.CURRENT_MODE) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        System.out.println("Starting AdvantageKit on a real robot");
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    try {
      Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
      watchdogField.setAccessible(true);
      Watchdog watchdog = (Watchdog) watchdogField.get(this);
      watchdog.setTimeout(loopOverrunWarningTimeout);
    } catch (Exception e) {
      DriverStation.reportWarning("Failed to disable loop overrun warnings.", false);
    }
    CommandScheduler.getInstance().setPeriod(loopOverrunWarningTimeout);

    // Silence controller disconnected warnings
    DriverStation.silenceJoystickConnectionWarning(true);

    // Silence Rotation2d warnings
    var mathShared = MathSharedStore.getMathShared();
    MathSharedStore.setMathShared(
        new MathShared() {
          @Override
          public void reportError(String error, StackTraceElement[] stackTrace) {
            if (error.startsWith("x and y components of Rotation2d are zero")) {
              return;
            }
            mathShared.reportError(error, stackTrace);
          }

          @Override
          public void reportUsage(MathUsageId id, int count) {
            mathShared.reportUsage(id, count);
          }

          @Override
          public double getTimestamp() {
            return mathShared.getTimestamp();
          }
        });

    // Log active commands
    Map<String, Integer> commandCounts = new HashMap<>();
    BiConsumer<Command, Boolean> logCommandFunction =
        (Command command, Boolean active) -> {
          String name = command.getName();
          int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
          commandCounts.put(name, count);
          Logger.recordOutput(
              "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
          Logger.recordOutput("CommandsAll/" + name, count > 0);
        };

    RobotController.setBrownoutVoltage(6.0);
    SignalLogger.enableAutoLogging(false);

    // We use Rust now
    HAL.report(tResourceType.kResourceType_Language, tInstances.kLanguage_Rust);

    CommandScheduler.getInstance()
        .onCommandInitialize((Command command) -> logCommandFunction.accept(command, true));
    CommandScheduler.getInstance()
        .onCommandFinish((Command command) -> logCommandFunction.accept(command, false));
    CommandScheduler.getInstance()
        .onCommandInterrupt((Command command) -> logCommandFunction.accept(command, false));

    robotContainer = new RobotContainer();

    machine.addTransition(testStates.STATE_1, testStates.STATE_2, new PrintCommand("1 -> 2"));
    machine.addStateCommand(testStates.STATE_1, new PrintCommand("1"));
    machine.addStateCommand(testStates.STATE_2, new PrintCommand("2"));
    machine.addStateCommand(testStates.STATE_3, new PrintCommand("3"));
    machine.addStateCommand(testStates.STATE_4, new PrintCommand("4"));

    machine.addDualTransition(testStates.STATE_2, testStates.STATE_4, new PrintCommand("2 -> 4"));
    machine.addTransition(testStates.STATE_4, testStates.STATE_3, new PrintCommand("4 -> 3"));
    machine.addTransition(testStates.STATE_3, testStates.STATE_1, new PrintCommand("4 -> 3"));
    machine.enable();
    System.out.println(machine.dot());
  }

  @Override
  public void robotPeriodic() {
    // Refresh signals before subsystem periodics
    ExecutionLogger.reset();
    PhoenixUtils.refreshSignals();
    ExecutionLogger.log("PhoenixSignalRefresh");

    CommandScheduler.getInstance().run();
    ExecutionLogger.log("Commands");

    robotContainer.updateAlerts();

    machine.requestTransition(testStates.STATE_3);
    machine.periodic();
    ExecutionLogger.log("RobotPeriodic");
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
