package org.team157.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.team157.robot.Constants.TelemetryConstants;
import org.team157.utilities.PosUtils;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.local.SparkWrapper;

/**
 * Hood running on a SPARK MAX, with a REV Through Bore absolute encoder on a roboRIO DIO port.
 *
 * <p>YAMS's SparkWrapper can only use a SPARK absolute encoder for external feedback, so the
 * roboRIO encoder is not used for closed loop control directly. Instead the motor's built-in
 * encoder runs the closed loop and is seeded from the absolute encoder's angle.
 */
public class HoodIOSparkMax implements HoodIO {

  private final Pivot hood;
  private final SmartMotorController motor;
  // Returns 0 to 1 rotations
  private final DutyCycleEncoder encoder =
      new DutyCycleEncoder(HoodConstants.ENCODER_ID, 1.0, HoodConstants.ENCODER_ZERO_OFFSET);

  public HoodIOSparkMax(SubsystemBase subsystem) {
    SparkMax sparkmax = new SparkMax(HoodConstants.MOTOR_ID, MotorType.kBrushless);

    SmartMotorControllerConfig hoodMotorConfig =
        new SmartMotorControllerConfig(subsystem)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withTelemetry("HoodMotor", TelemetryConstants.TELEMETRY_VERBOSITY)
            .withClosedLoopController(HoodConstants.KP, HoodConstants.KI, HoodConstants.KD)
            .withSimClosedLoopController(
                HoodConstants.SIM_KP, HoodConstants.SIM_KI, HoodConstants.SIM_KD)
            .withTrapezoidalProfile(HoodConstants.MAX_VELOCITY, HoodConstants.MAX_ACCELERATION)
            .withIdleMode(MotorMode.BRAKE)
            // Inverted so positive output raises the hood angle (retracts toward 42.5 degrees).
            // Checked on the robot: before inverting, positive output (Start) lowered the angle.
            .withMotorInverted(true)
            .withGearing(HoodConstants.GEARING)
            .withSoftLimit(HoodConstants.LOWER_SOFT_LIMIT, HoodConstants.UPPER_SOFT_LIMIT)
            .withStatorCurrentLimit(HoodConstants.CURRENT_LIMIT)
            .withClosedLoopRampRate(HoodConstants.RAMP_RATE);

    // Create the hood's motor controller with the above configuration.
    SmartMotorController smartHoodMotor =
        new SparkWrapper(sparkmax, DCMotor.getNEO(1), hoodMotorConfig);

    // Configure the physical characteristics of the hood.
    PivotConfig hoodConfig =
        new PivotConfig(smartHoodMotor)
            .withTelemetry("Hood", TelemetryConstants.TELEMETRY_VERBOSITY)
            .withStartingPosition(HoodConstants.UPPER_SOFT_LIMIT)
            .withHardLimit(HoodConstants.LOWER_HARD_LIMIT, HoodConstants.UPPER_HARD_LIMIT)
            .withMOI(Meters.of(0.2), Kilograms.of(0.5));

    // Create the hood pivot system with the above configuration.
    this.hood = new Pivot(hoodConfig);
    this.motor = hood.getMotor();

    seedEncoder();
  }

  /**
   * Converts the absolute encoder's raw reading to a hood angle using the readings measured at each
   * hard stop.
   */
  private Angle getEncoderAngle() {
    return Degrees.of(
        PosUtils.mapRange(
            encoder.get(),
            HoodConstants.ENCODER_AT_LOWER_HARD_LIMIT,
            HoodConstants.ENCODER_AT_UPPER_HARD_LIMIT,
            HoodConstants.LOWER_HARD_LIMIT.in(Degrees),
            HoodConstants.UPPER_HARD_LIMIT.in(Degrees)));
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.supplyCurrentAmps = motor.getSupplyCurrent().map(c -> c.in(Amps)).orElse(0.0);
    inputs.statorCurrentAmps = motor.getStatorCurrent().in(Amps);
    inputs.appliedVolts = motor.getVoltage().in(Volts);
    inputs.temperatureCelsius = motor.getTemperature().in(Celsius);
    inputs.angleDegrees = hood.getAngle().in(Degrees);
    inputs.targetAngleDegrees =
        motor.getMechanismPositionSetpoint().map(a -> a.in(Degrees)).orElse(0.0);
    inputs.encoderPositionRotations = encoder.get();
    inputs.encoderConnected = encoder.isConnected();
    inputs.angleFromEncoderDegrees = getEncoderAngle().in(Degrees);
    inputs.mechanismVelocityDegreesPerSecond = motor.getMechanismVelocity().in(DegreesPerSecond);
  }

  @Override
  public void seedEncoder() {
    // An unplugged DutyCycleEncoder reads 0, which would seed a wrong angle
    if (RobotBase.isReal() && encoder.isConnected()) {
      motor.setEncoderPosition(getEncoderAngle());
    }
  }

  @Override
  public Command setTargetAngle(Angle angle) {
    return hood.setAngle(angle);
  }

  @Override
  public Command setTargetAngle(Supplier<Angle> angle) {
    return hood.setAngle(angle);
  }

  @Override
  public Command stop() {
    return hood.set(0);
  }

  @Override
  public Command set(double dutycycle) {
    return hood.set(dutycycle);
  }

  @Override
  public void simIterate() {
    hood.simIterate();
  }
}
