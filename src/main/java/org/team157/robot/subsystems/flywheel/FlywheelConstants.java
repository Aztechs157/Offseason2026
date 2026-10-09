// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;

public final class FlywheelConstants {
  // IDs of both motors powering the flywheel.
  public static final int MOTOR_ID_RIGHT = 51,
      FOLLOWER_MOTOR_ID_RIGHT = 52,
      FOLLOWER_MOTOR_ID_LEFT_1 = 56,
      FOLLOWER_MOTOR_ID_LEFT_2 = 57;
  // Closed-loop control values for the flywheel.
  public static final double KP = 2, KI = 0, KD = 0;
  public static final double KS = 0.0, KV = 0.0, KA = 0.0;
  public static final double SIM_KP = 1.57, SIM_KI = 0, SIM_KD = 0.157;
  public static final double SIM_KS = 0.0,
      SIM_KV = 0.37,
      SIM_KA = 0.27; // Determined via Reca.lc, tune and/or run
  // SysID to verify these values.
  public static final AngularVelocity ANGULAR_VELOCITY = RPM.of(5800);
  public static final AngularAcceleration ANGULAR_ACCELERATION =
      RotationsPerSecondPerSecond.of(11600);
  // Gear ratio between the motor and the flywheel.
  public static final MechanismGearing GEARING = new MechanismGearing(GearBox.fromStages("32:22"));
  // Diameter of the flywheel, in meters.
  public static final Distance FLYWHEEL_DIAMETER = Inches.of(6);
  // Mass of the flywheel (including shooter wheels), in pounds.
  public static final Mass FLYWHEEL_MASS = Pounds.of(1);
  // Z distance from the center of the flywheel to the ground, in meters.
  public static final Distance HEIGHT = Meters.of(0.523);

  public static final Time RAMP_RATE = Seconds.of(0.00157);
  // Flywheel RPM limits for safety, in... RPM.
  public static final AngularVelocity FLYWHEEL_RPM_LIMIT_UPPER = RPM.of(5800),
      FLYWHEEL_RPM_LIMIT_LOWER = RPM.of(-5800);
  // RPM multiplier to account for external factors like air resistance and wheel slip. This is
  // determined experimentally.
  public static final double SPEED_FACTOR = 1 / 0.385;
  // Flywheel speed when shooting with dumper mode off. TODO: tune
  public static final AngularVelocity MANUAL_SHOT_VELOCITY = RPM.of(1000);
  // Limits for the calculated (dumper mode) flywheel speed. The top limit is a bit under the
  // flywheel's free speed (NEO through 32:22 is about 3900 RPM). TODO: tune
  public static final AngularVelocity MIN_DYNAMIC_VELOCITY = RPM.of(1000),
      MAX_DYNAMIC_VELOCITY = RPM.of(3600);
  // How close the flywheel must be to its target speed before balls are fed in to shoot.
  public static final AngularVelocity VELOCITY_TOLERANCE = RPM.of(100);
  // Max time to wait for the flywheel to spin up (and the robot to aim) before feeding anyway.
  public static final Time SPIN_UP_TIMEOUT = Seconds.of(1.5);
  public static final Current CURRENT_LIMIT = Amps.of(40);
  // SysId settings: voltage for the dynamic tests, how fast the voltage climbs in the quasistatic
  // tests, and the max time for each test.
  public static final Voltage SYSID_STEP_VOLTAGE = Volts.of(7);
  public static final Velocity<VoltageUnit> SYSID_RAMP_RATE = Volts.of(1).per(Second);
  public static final Time SYSID_TEST_DURATION = Seconds.of(8);
}
