package org.team157.robot.subsystems.hood;

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

public class HoodConstants {
  /** Hood SPARK MAX CAN ID */
  public static final int MOTOR_ID = 54;

  /** roboRIO DIO port for the hood's REV Through Bore encoder (absolute/PWM output) */
  public static final int ENCODER_ID = 0;

  /** 3:1 MAXPlanetary slice on the motor, then 1:4 from the gearbox output to the hood */
  public static final MechanismGearing GEARING =
      new MechanismGearing(GearBox.fromStages("3:1", "1:4"));

  /** Physical hard stops of the hood. TODO: measure on robot (these are from the 2026 robot) */
  public static final Angle LOWER_HARD_LIMIT = Degrees.of(40), UPPER_HARD_LIMIT = Degrees.of(65);

  /** How far inside each hard stop the soft limits sit, so the hood doesn't slam into them */
  public static final Angle SOFT_LIMIT_MARGIN = Degrees.of(2);

  /** Soft limits for the hood, enforced by the motor controller */
  public static final Angle LOWER_SOFT_LIMIT = LOWER_HARD_LIMIT.plus(SOFT_LIMIT_MARGIN),
      UPPER_SOFT_LIMIT = UPPER_HARD_LIMIT.minus(SOFT_LIMIT_MARGIN);

  /**
   * Raw encoder readings (0 to 1 rotations) with the hood pushed against each hard stop. TODO: read
   * "Hood/EncoderPositionRotations" in AdvantageScope with the hood pushed to each stop.
   */
  public static final double ENCODER_AT_LOWER_HARD_LIMIT = 0.32, ENCODER_AT_UPPER_HARD_LIMIT = 0.86;

  /**
   * Shifts where the encoder's reading wraps from 1 back to 0. If the reading jumps between 0 and 1
   * anywhere in the hood's travel, set this to move the jump outside of it (e.g. 0.5 moves it half
   * a turn), then re-measure the readings above.
   */
  public static final double ENCODER_ZERO_OFFSET = 0;

  /**
   * If the motor encoder and absolute encoder disagree by more than this while disabled, the motor
   * encoder is reseeded from the absolute encoder.
   */
  public static final Angle ENCODER_RESYNC_THRESHOLD = Degrees.of(1);

  /**
   * PID values for the hood, run on the roboRIO by YAMS every 20ms. Units are volts per rotation of
   * the hood, so a KP of 2 applies about 0.0056V per degree of error. TODO: tune
   *
   * <p>The 2026 robot's KP of 157 was for a Kraken X44 through 36.6:1. This hood's NEO through
   * 0.75:1 moves about 37x faster per volt, so KP has to be much smaller. Each 1 of KP corrects
   * about 21% of the error every 20ms loop.
   *
   * <p>Tuning: graph Hood/AngleDegrees against Hood/TargetAngleDegrees in AdvantageScope.
   *
   * <ul>
   *   <li>KP: start at 2. If the hood stops short of the target or moves slowly, raise it in steps
   *       (3, 4). If it buzzes or overshoots, lower it. Stay under about 4.5, where it starts
   *       overshooting every loop. Around 9.5 it oscillates.
   *   <li>KD: start at 0. Add small amounts (0.02 to 0.05) if a higher KP overshoots.
   *   <li>KI: leave at 0. If the hood always stops a little short, it needs feedforward (kS for
   *       friction, kG for the hood's weight), not KI.
   * </ul>
   */
  public static final double KP = 2, KI = 0, KD = 0;

  public static final double SIM_KP = 20, SIM_KI = 0, SIM_KD = 0;

  /** Trapezoidal motion profile limits for the hood */
  public static final AngularVelocity MAX_VELOCITY = DegreesPerSecond.of(360);

  public static final AngularAcceleration MAX_ACCELERATION = DegreesPerSecondPerSecond.of(360);

  public static final Current CURRENT_LIMIT = Amps.of(40);
  public static final Time RAMP_RATE = Seconds.of(0.00157);
}
