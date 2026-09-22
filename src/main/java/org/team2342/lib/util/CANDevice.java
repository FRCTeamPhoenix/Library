// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.util;

import com.ctre.phoenix6.CANBus;
import lombok.Getter;
import org.wpilib.hardware.bus.CANPort;

public class CANDevice {

  @Getter private final int deviceNumber;
  @Getter private final CANPort port;

  public CANDevice(int deviceNumber, CANPort port) {
    this.deviceNumber = deviceNumber;
    this.port = port;
  }

  public CANBus getCANBus() {
    return new CANBus(port);
  }
}
