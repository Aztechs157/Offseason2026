package org.team157.robot.subsystems.intakeDeploy;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.InchesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;

/**
 * Constants for the rack and pinion intake deploy. All positions are measured as distance of rack
 * travel from the fully retracted (starting) position, so 0 in = inside the robot and positive =
 * deployed outward toward the floor.
 */
public class IntakeDeployConstants {
  /** Rack motor CAN ID and potentiometer roboRIO analog input channel */
  public static final int RACK_MOTOR_ID = 58, RACK_POTENTIOMETER_ID = 0;

  /** Gearing from the motor to the pinion */
  public static final MechanismGearing RACK_GEARING =
      new MechanismGearing(GearBox.fromStages("9:1", "24:18"));

  /**
   * Pinion tooth count and diametral pitch (DP, teeth per inch of pitch diameter). DP is usually
   * printed on the gear or listed by the vendor.
   */
  public static final double PINION_TEETH = 14, PINION_DIAMETRAL_PITCH = 10;

  /** Pitch diameter of the pinion, where its teeth mesh with the rack (teeth / DP) */
  public static final Distance PINION_PITCH_DIAMETER =
      Inches.of(PINION_TEETH / PINION_DIAMETRAL_PITCH);

  /** Distance the rack moves per pinion rotation (pitch diameter * pi) */
  public static final Distance PINION_CIRCUMFERENCE = PINION_PITCH_DIAMETER.times(Math.PI);

  /**
   * PID values for the rack, run on the roboRIO by YAMS. Units are volts per meter of error, so a
   * KP of 40 applies about 1V when the rack is 1 inch off its target. TODO: tune
   *
   * <p>Tuning: graph IntakeDeploy/PositionInches against IntakeDeploy/TargetPositionInches in
   * AdvantageScope. Tune the feedforward first so the PID only has to correct small errors.
   *
   * <ul>
   *   <li>KP: start at 40. If the rack lags behind the target or stops short, raise it in steps
   *       (80, 120, 160). If it overshoots or buzzes back and forth, lower it.
   *   <li>KD: start at 0. If a KP high enough to reach the target overshoots, add KD in small steps
   *       (0.5 to 2) to damp it.
   *   <li>KI: leave at 0. A correct kG should remove any steady error caused by the slope.
   * </ul>
   */
  public static final double KP = 40, KI = 0, KD = 0;

  /** PID for simulation only. The sim has no gravity or friction, so these won't match the robot */
  public static final double SIM_KP = 40, SIM_KI = 0, SIM_KD = 0;

  /**
   * Feedforward for the rack (kS, kG, kV), in volts and meters per second. TODO: tune
   *
   * <ul>
   *   <li>kG: start at -0.2. Calculated from the 12.84 lb intake on the -31.7 degree slope through
   *       12:1 gearing. It is negative because gravity pulls the rack outward (positive), and
   *       ElevatorFeedforward assumes gravity pulls toward negative. Check on the robot: the
   *       voltage that just holds the rack still on the slope is about kG.
   *   <li>kS: start at 0. Raise it until the rack barely begins moving from a stop in both
   *       directions. Friction in the rack may need 0.1 to 0.5V.
   *   <li>kV: start at 13.6. Calculated from NEO free speed through 12:1 gearing and the 4.396 in
   *       pinion, so 25 in/s needs about 8.6V. If the rack lags the target while moving at full
   *       speed, raise it slightly. If it runs ahead of the target, lower it.
   * </ul>
   */
  public static final ElevatorFeedforward FEEDFORWARD = new ElevatorFeedforward(0, 0, 0);

  /** Trapezoidal motion profile limits for the rack */
  public static final LinearVelocity MAX_VELOCITY = InchesPerSecond.of(25);

  public static final LinearAcceleration MAX_ACCELERATION = InchesPerSecondPerSecond.of(120);

  /** Physical hard stops of the rack (full travel) */
  public static final Distance LOWER_HARD_LIMIT = Inches.of(0), UPPER_HARD_LIMIT = Inches.of(20.59);

  /** How far inside each hard stop the soft limits sit, so the rack doesn't slam into them */
  public static final Distance SOFT_LIMIT_MARGIN = Inches.of(0.25);

  /** Soft limits for the rack, enforced by the motor controller */
  public static final Distance LOWER_SOFT_LIMIT = LOWER_HARD_LIMIT.plus(SOFT_LIMIT_MARGIN),
      UPPER_SOFT_LIMIT = UPPER_HARD_LIMIT.minus(SOFT_LIMIT_MARGIN);

  /** How far the intake pulls back from deployed on each wiggle. TODO: tune */
  public static final Distance WIGGLE_DISTANCE = Inches.of(6);

  /** Setpoints for the rack. Must be between the soft limits. */
  public static final Distance RETRACTED_POSITION = LOWER_SOFT_LIMIT,
      DEPLOYED_POSITION = UPPER_SOFT_LIMIT,
      WIGGLE_IN_POSITION = DEPLOYED_POSITION.minus(WIGGLE_DISTANCE);

  /** How close the rack must be to a setpoint to be considered there */
  public static final Distance POSITION_TOLERANCE = Inches.of(0.25);

  /** Max time to spend on a full deploy/retract, or on one move in or out during a wiggle */
  public static final Time MOVE_TIMEOUT = Seconds.of(2), WIGGLE_MOVE_TIMEOUT = Seconds.of(0.6);

  /**
   * Raw potentiometer readings (0 to 1, fraction of 5V) with the rack pushed against each hard
   * stop. The 10 turn pot is geared 3:4 off the pinion, which these two readings account for. TODO:
   * read "IntakeDeploy/PotRaw" in AdvantageScope with the intake pushed to each stop.
   */
  public static final double POT_AT_LOWER_HARD_LIMIT = 0.40, POT_AT_UPPER_HARD_LIMIT = 0.60;

  /**
   * If the motor encoder and pot disagree by more than this while disabled, the encoder is reseeded
   * from the pot.
   */
  public static final Distance ENCODER_RESYNC_THRESHOLD = Inches.of(0.25);

  /** Slope of the rack from horizontal, negative = deploying moves down. Used for visualization */
  public static final Angle RACK_ANGLE = Degrees.of(-31.732522);

  /** Mass of the moving intake assembly, used for simulation */
  public static final Mass INTAKE_MASS = Pounds.of(12.8389624);

  /** Current limit for the rack motor */
  public static final Current CURRENT_LIMIT = Amps.of(40);

  /** Ramp rate for the rack motor */
  public static final Time RAMP_RATE = Seconds.of(0.1);
}
