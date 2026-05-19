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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.team157.robot.subsystems.hood.HoodIO.HoodIOInputs;
import org.team157.utilities.PosUtils;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.local.SparkWrapper;

public class HoodIOSparkMax implements HoodIO {

  private final Pivot hood;
  private final SmartMotorController motor;
  private final DutyCycleEncoder encoder;

  public HoodIOSparkMax(SubsystemBase subsystem) {
    this.encoder = new DutyCycleEncoder(HoodConstants.ENCODER_ID);
    SparkMax sparkmax = new SparkMax(157, MotorType.kBrushless);

    SmartMotorControllerConfig hoodMotorConfig =
        new SmartMotorControllerConfig(subsystem)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(
                HoodConstants.KP,
                HoodConstants.KI,
                HoodConstants.KD,
                HoodConstants.ANGULAR_VELOCITY,
                HoodConstants.ANGULAR_ACCELERATION)
            .withSimClosedLoopController(
                HoodConstants.SIM_KP,
                HoodConstants.SIM_KI,
                HoodConstants.SIM_KD,
                HoodConstants.ANGULAR_VELOCITY,
                HoodConstants.ANGULAR_ACCELERATION)
            .withIdleMode(MotorMode.BRAKE)
            .withMotorInverted(false)
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
            // .withStartingPosition(
            //     Degrees.of(mapHoodEncoder(HoodConstants.MIN_ANGLE, HoodConstants.MAX_ANGLE)))
            .withHardLimit(HoodConstants.LOWER_HARD_LIMIT, HoodConstants.UPPER_HARD_LIMIT)
            .withMOI(Meters.of(0.2), Kilograms.of(0.5));

    // Create the hood pivot system with the above configuration.
    this.hood = new Pivot(hoodConfig);
    this.motor = hood.getMotor();
  }

  /**
   * Helper function that maps the hood's current encoder value to a given range using PosUtils.
   *
   * @param min The minimum value of the remapped range (equivalent to the hood's minimum encoder
   *     position)
   * @param max The maximum value of the remapped range (equivalent to hood's maximum encoder
   *     position)
   * @return The current value of the hood's encoder, mapped between 2 numbers based on the
   *     configured minimum and maximum encoder values.
   */
  private double mapHoodEncoder(double min, double max) {
    return PosUtils.mapRange(
        encoder.get(),
        HoodConstants.MIN_ENCODER_POSITION,
        HoodConstants.MAX_ENCODER_POSITION,
        min,
        max);
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
    inputs.angleFromEncoderDegrees =
        mapHoodEncoder(HoodConstants.MIN_ANGLE, HoodConstants.MAX_ANGLE);
    inputs.mechanismVelocityDegreesPerSecond = motor.getMechanismVelocity().in(DegreesPerSecond);
    inputs.scaledEncoderPosition = mapHoodEncoder(0, 1);
  }

  @Override
  public Command setTargetAngle(Angle angle) {
    return hood.setAngle(angle).finallyDo(() -> stop());
  }

  @Override
  public Command setTargetAngle(Supplier<Angle> angle) {
    return hood.setAngle(angle).finallyDo(() -> stop());
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
