package org.team157.robot.subsystems.intakeDeploy;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;

public class IntakeDeployConstants {
  /** RACK motor and encoder IDs */
  public static final int RACK_MOTOR_ID = 58, RACK_POTENTIOMETER_ID = 0;
  /** Gearing for the RACK motor */
  public static final MechanismGearing RACK_GEARING =
      new MechanismGearing(GearBox.fromStages("4:1", "24:18"));
  /** Closed loop control values for the RACK */
  public static final double KP = 157, KI = 0, KD = 0;

  public static final double SIM_KP = 20, SIM_KI = 0, SIM_KD = 0;
  /** Angular velocity for the RACK motor */
  public static final AngularVelocity ANGULAR_VELOCITY = DegreesPerSecond.of(360);
  /** Angular acceleration for the RACK motor */
  public static final AngularAcceleration ANGULAR_ACCELERATION = DegreesPerSecondPerSecond.of(540);
  /** Minimum and maximum positions for the RACK encoder */
  public static final double MIN_POT_POSITION = 0.76, MAX_POT_POSITION = 0.35;
  /** Minimum and maximum angles for the RACK */
  public static final double MIN_ANGLE = 0, MAX_ANGLE = 80;
  /** Soft limits for the RACK angle */
  public static final Angle LOWER_SOFT_LIMIT = Degrees.of(-500), UPPER_SOFT_LIMIT = Degrees.of(80);
  /** Hard limits for the RACK angle */
  public static final Angle LOWER_HARD_LIMIT = Degrees.of(-500), UPPER_HARD_LIMIT = Degrees.of(80);
  /** Current limit for the RACK motor */
  public static final Current CURRENT_LIMIT = Amps.of(40);
  /** Ramp rate for the RACK motor */
  public static final Time RAMP_RATE = Seconds.of(0.00157);
  /** Duty cycle to hold the RACK in place */
  public static final double RACK_HOLD = 0.0157;
}
