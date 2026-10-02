// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.team157.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.RobotBase;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static class TelemetryConstants {
    public static final TelemetryVerbosity TELEMETRY_VERBOSITY = TelemetryVerbosity.LOW;
  }

  public static final CANBus RIO_CAN_BUS = new CANBus("rio", "./logs/example.hoot");
  public static final CANBus DRIVE_CAN_BUS = new CANBus("canivore", "./logs/example2.hoot");
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
