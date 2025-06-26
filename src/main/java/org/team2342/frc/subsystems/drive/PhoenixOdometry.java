// Copyright (c) 2025 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.team2342.frc.Constants.DriveConstants;

/** Reads high-frequency measurements into queues for odometry. */
@SuppressWarnings("unused")
public class PhoenixOdometry extends Thread {
  private final Lock signalLock = new ReentrantLock();

  private BaseStatusSignal[] signals = new BaseStatusSignal[0];

  private final List<Queue<Double>> measurementQueues = new ArrayList<>();
  private final List<Queue<Double>> timestampQueues = new ArrayList<>();

  private static PhoenixOdometry instance = null;

  public static PhoenixOdometry getInstance() {
    if (instance == null) instance = new PhoenixOdometry();
    return instance;
  }

  private PhoenixOdometry() {
    setName("PhoenixOdometryThread");
    setDaemon(true);
  }

  @Override
  public void start() {
    if (timestampQueues.size() > 0) super.start();
  }

  /** Registers a signal to be read. */
  public Queue<Double> registerSignal(BaseStatusSignal signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    signalLock.lock();
    Drive.odometryLock.lock();
    try {
      BaseStatusSignal[] newSignals = new BaseStatusSignal[signals.length + 1];
      System.arraycopy(signals, 0, newSignals, 0, signals.length);
      newSignals[signals.length] = signal;
      signals = newSignals;
      measurementQueues.add(queue);
    } finally {
      signalLock.unlock();
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  /** Returns a new timestamp queue. */
  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drive.odometryLock.lock();
    try {
      timestampQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  @Override
  public void run() {
    while (true) {
      signalLock.lock();
      try {
        if (DriveConstants.IS_CANFD && signals.length > 0) {
          BaseStatusSignal.waitForAll(2.0 / DriveConstants.ODOMETRY_FREQUENCY, signals);
        } else {
          Thread.sleep((long) (1000.0 / DriveConstants.ODOMETRY_FREQUENCY));
          if (signals.length > 0) BaseStatusSignal.refreshAll(signals);
        }
      } catch (Exception e) {
        e.printStackTrace();
      } finally {
        signalLock.unlock();
      }

      Drive.odometryLock.lock();
      try {
        double timestamp = RobotController.getFPGATime() / 1e6;
        double totalLatency = 0.0;
        for (BaseStatusSignal signal : signals) {
          totalLatency += signal.getTimestamp().getLatency();
        }
        if (signals.length > 0) {
          timestamp -= totalLatency / signals.length;
        }

        for (int i = 0; i < signals.length; i++) {
          measurementQueues.get(i).offer(signals[i].getValueAsDouble());
        }
        for (int i = 0; i < timestampQueues.size(); i++) {
          timestampQueues.get(i).offer(timestamp);
        }
      } finally {
        Drive.odometryLock.unlock();
      }
    }
  }
}
