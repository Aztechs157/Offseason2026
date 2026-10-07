package org.team157.robot.subsystems.intakeDeploy;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.team157.utilities.PosUtils;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.local.SparkWrapper;

public class IntakeDeployIOSparkFlex implements IntakeDeployIO {
  private final Pivot slapdown;
  private final SmartMotorController motor;
  private final AnalogPotentiometer potentiometer =
      new AnalogPotentiometer(
          new AnalogInput(IntakeDeployConstants.RACK_POTENTIOMETER_ID), 500, 30);

  public IntakeDeployIOSparkFlex(SubsystemBase subsystem) {
    SparkFlex sparkFlex = new SparkFlex(IntakeDeployConstants.RACK_MOTOR_ID, MotorType.kBrushless);

    // Step 1: Create SmartMotorControllerConfig
    SmartMotorControllerConfig smcConfig =
        new SmartMotorControllerConfig(subsystem)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(
                IntakeDeployConstants.KP,
                IntakeDeployConstants.KI,
                IntakeDeployConstants.KD,
                IntakeDeployConstants.ANGULAR_VELOCITY,
                IntakeDeployConstants.ANGULAR_ACCELERATION)
            .withSimClosedLoopController(
                IntakeDeployConstants.SIM_KP,
                IntakeDeployConstants.SIM_KI,
                IntakeDeployConstants.SIM_KD,
                IntakeDeployConstants.ANGULAR_VELOCITY,
                IntakeDeployConstants.ANGULAR_ACCELERATION)
            .withIdleMode(MotorMode.BRAKE)
            .withMotorInverted(true)
            .withGearing(IntakeDeployConstants.RACK_GEARING)
            .withStatorCurrentLimit(IntakeDeployConstants.CURRENT_LIMIT)
            .withClosedLoopRampRate(IntakeDeployConstants.RAMP_RATE);

    // Create the pivot's motor controller with the above configuration.
    SmartMotorController smc = new SparkWrapper(sparkFlex, DCMotor.getNEO(1), smcConfig);

    // Configure the physical characteristics of the pivot.
    PivotConfig slapdownConfig =
        new PivotConfig(smc)
            .withStartingPosition(Degrees.of(80))
            .withHardLimit(
                IntakeDeployConstants.LOWER_HARD_LIMIT, IntakeDeployConstants.UPPER_HARD_LIMIT)
            .withSoftLimits(
                IntakeDeployConstants.LOWER_SOFT_LIMIT, IntakeDeployConstants.UPPER_SOFT_LIMIT)
            .withMOI(Meters.of(0.75), Kilograms.of(1));

    // Create the pivot system with the above configuration.
    this.slapdown = new Pivot(slapdownConfig);
    this.motor = slapdown.getMotor();
  }
  /**
   * Helper function that maps the pivot's current encoder value to a given range using PosUtils.
   *
   * @param min The minimum value of the remapped range (equivalent to the pivot's minimum encoder
   *     position)
   * @param max The maximum value of the remapped range (equivalent to pivot's maximum encoder
   *     position)
   * @return The current value of the hood's encoder, mapped between 2 numbers based on the
   *     configured minimum and maximum encoder values.
   */
  private double mapEncoder(double min, double max) {
    return PosUtils.mapRange(
        potentiometer.get(),
        IntakeDeployConstants.MIN_POT_POSITION,
        IntakeDeployConstants.MAX_POT_POSITION,
        min,
        max);
  }

  @Override
  public void updateInputs(SlapdownIOInputs inputs) {
    inputs.supplyCurrentAmps = motor.getSupplyCurrent().map(c -> c.in(Amps)).orElse(0.0);
    inputs.statorCurrentAmps = motor.getStatorCurrent().in(Amps);
    inputs.appliedVolts = motor.getVoltage().in(Volts);
    inputs.temperatureCelsius = motor.getTemperature().in(Celsius);
    inputs.angleDegrees = slapdown.getAngle().in(Degrees);
    inputs.targetAngleDegrees =
        motor.getMechanismPositionSetpoint().map(a -> a.in(Degrees)).orElse(0.0);
    inputs.encoderPositionRotations = potentiometer.get();
    inputs.angleFromEncoderDegrees =
        mapEncoder(IntakeDeployConstants.MIN_ANGLE, IntakeDeployConstants.MAX_ANGLE);
    inputs.mechanismVelocityDegreesPerSecond = motor.getMechanismVelocity().in(DegreesPerSecond);
    inputs.scaledEncoderPosition = mapEncoder(0, 1);
    inputs.isInStartingPosition = (inputs.angleDegrees > 60);
  }

  // TODO: Evaluate whether or not this method is necessary, as the Hood only needed it for
  // dynamic
  // angle setting.
  @Override
  public Command setTargetAngle(Angle angle) {
    return slapdown.setAngle(angle).finallyDo(() -> stop());
  }

  @Override
  public Command setTargetAngle(Supplier<Angle> angle) {
    return slapdown.setAngle(angle).finallyDo(() -> stop());
  }

  @Override
  public Command stop() {
    return slapdown.set(0);
  }

  @Override
  public Command set(double dutycycle) {
    return slapdown.set(dutycycle);
  }

  @Override
  public void simIterate() {
    slapdown.simIterate();
  }
}
