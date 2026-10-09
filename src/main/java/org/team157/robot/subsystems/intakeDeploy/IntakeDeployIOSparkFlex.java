package org.team157.robot.subsystems.intakeDeploy;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team157.robot.Constants.TelemetryConstants;
import org.team157.utilities.PosUtils;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.local.SparkWrapper;

/**
 * Rack and pinion intake deploy, modeled in YAMS as an {@link Elevator} so positions are linear
 * distances of rack travel.
 *
 * <p>YAMS's SparkWrapper can only use a SPARK absolute encoder for external feedback, so the
 * potentiometer (on the roboRIO) is not used for closed loop control directly. Instead the motor's
 * built-in encoder runs the closed loop and is seeded from the potentiometer's absolute position.
 */
public class IntakeDeployIOSparkFlex implements IntakeDeployIO {
  private final Elevator rack;
  private final SmartMotorController motor;
  // Returns 0 to 1 as a fraction of the 5V rail
  private final AnalogPotentiometer potentiometer =
      new AnalogPotentiometer(IntakeDeployConstants.RACK_POTENTIOMETER_ID);

  public IntakeDeployIOSparkFlex(SubsystemBase subsystem) {
    SparkFlex sparkFlex = new SparkFlex(IntakeDeployConstants.RACK_MOTOR_ID, MotorType.kBrushless);

    // Mechanism circumference must be set before the soft limits, otherwise YAMS throws.
    SmartMotorControllerConfig smcConfig =
        new SmartMotorControllerConfig(subsystem)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withTelemetry("IntakeDeployMotor", TelemetryConstants.TELEMETRY_VERBOSITY)
            .withGearing(IntakeDeployConstants.RACK_GEARING)
            .withMechanismCircumference(IntakeDeployConstants.PINION_CIRCUMFERENCE)
            .withClosedLoopController(
                IntakeDeployConstants.KP, IntakeDeployConstants.KI, IntakeDeployConstants.KD)
            .withSimClosedLoopController(
                IntakeDeployConstants.SIM_KP,
                IntakeDeployConstants.SIM_KI,
                IntakeDeployConstants.SIM_KD)
            .withTrapezoidalProfile(
                IntakeDeployConstants.MAX_VELOCITY, IntakeDeployConstants.MAX_ACCELERATION)
            .withFeedforward(IntakeDeployConstants.FEEDFORWARD)
            .withSoftLimit(
                IntakeDeployConstants.LOWER_SOFT_LIMIT, IntakeDeployConstants.UPPER_SOFT_LIMIT)
            .withIdleMode(MotorMode.BRAKE)
            // Raw negative output deploys (checked in REV Hardware Client), so invert to make
            // positive output and positive positions mean deploying outward

            .withMotorInverted(true)
            .withStatorCurrentLimit(IntakeDeployConstants.CURRENT_LIMIT)
            .withClosedLoopRampRate(IntakeDeployConstants.RAMP_RATE)
            .withOpenLoopRampRate(IntakeDeployConstants.RAMP_RATE);

    SmartMotorController smc = new SparkWrapper(sparkFlex, DCMotor.getNEO(1), smcConfig);

    // The rack is treated as a horizontal elevator so the sim doesn't apply full vertical gravity.
    ElevatorConfig rackConfig =
        new ElevatorConfig(smc)
            .withTelemetry("IntakeDeploy", TelemetryConstants.TELEMETRY_VERBOSITY)
            .withStartingHeight(IntakeDeployConstants.LOWER_HARD_LIMIT)
            .withHardLimits(
                IntakeDeployConstants.LOWER_HARD_LIMIT, IntakeDeployConstants.UPPER_HARD_LIMIT)
            .withAngle(IntakeDeployConstants.RACK_ANGLE)
            .withMass(IntakeDeployConstants.INTAKE_MASS)
            .withHorizontalElevator();

    this.rack = new Elevator(rackConfig);
    this.motor = rack.getMotor();

    seedEncoderFromPot();
  }

  /**
   * Converts the potentiometer's raw reading to a rack position using the readings measured at each
   * hard stop.
   */
  private Distance getPotPosition() {
    return Inches.of(
        PosUtils.mapRange(
            potentiometer.get(),
            IntakeDeployConstants.POT_AT_LOWER_HARD_LIMIT,
            IntakeDeployConstants.POT_AT_UPPER_HARD_LIMIT,
            IntakeDeployConstants.LOWER_HARD_LIMIT.in(Inches),
            IntakeDeployConstants.UPPER_HARD_LIMIT.in(Inches)));
  }

  @Override
  public void updateInputs(IntakeDeployIOInputs inputs) {
    inputs.supplyCurrentAmps = motor.getSupplyCurrent().map(c -> c.in(Amps)).orElse(0.0);
    inputs.statorCurrentAmps = motor.getStatorCurrent().in(Amps);
    inputs.appliedVolts = motor.getVoltage().in(Volts);
    inputs.temperatureCelsius = motor.getTemperature().in(Celsius);
    inputs.positionInches = rack.getHeight().in(Inches);
    inputs.targetPositionInches =
        motor
            .getMechanismPositionSetpoint()
            .map(a -> motor.getConfig().convertFromMechanism(a).in(Inches))
            .orElse(0.0);
    inputs.velocityInchesPerSecond = rack.getVelocity().in(InchesPerSecond);
    inputs.potRaw = potentiometer.get();
    inputs.potPositionInches = getPotPosition().in(Inches);
  }

  @Override
  public void seedEncoderFromPot() {
    if (RobotBase.isReal()) {
      motor.setEncoderPosition(getPotPosition());
    }
  }

  @Override
  public Command runTo(Distance position, Distance tolerance) {
    return rack.runTo(position, tolerance);
  }

  @Override
  public Command stop() {
    return rack.set(0);
  }

  @Override
  public Command set(double dutycycle) {
    return rack.set(dutycycle);
  }

  @Override
  public void simIterate() {
    rack.simIterate();
  }
}
