package org.team157.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.team157.robot.Constants.TelemetryConstants;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.local.SparkWrapper;

public class FlywheelIOSparkflex implements FlywheelIO {

  private final FlyWheel flywheel;
  private final SmartMotorController motor;
  private final SparkFlex sparkFlex;

  public FlywheelIOSparkflex(SubsystemBase subsystem) {
    this.sparkFlex = new SparkFlex(FlywheelConstants.MOTOR_ID_RIGHT, MotorType.kBrushless);
    SparkMax followerSparkMax_RIGHT =
        new SparkMax(FlywheelConstants.FOLLOWER_MOTOR_ID_RIGHT, MotorType.kBrushless);
    SparkMax followerSparkMax_LEFT_1 =
        new SparkMax(FlywheelConstants.FOLLOWER_MOTOR_ID_LEFT_1, MotorType.kBrushless);
    SparkMax followerSparkMax_LEFT_2 =
        new SparkMax(FlywheelConstants.FOLLOWER_MOTOR_ID_LEFT_2, MotorType.kBrushless);

    SmartMotorControllerConfig flywheelMotorConfig =
        new SmartMotorControllerConfig(subsystem)
            .withControlMode(ControlMode.CLOSED_LOOP)
            // A telemetry name is required by YAMS to create the SysId routine
            .withTelemetry("FlywheelMotor", TelemetryConstants.TELEMETRY_VERBOSITY)
            .withClosedLoopController(
                FlywheelConstants.KP,
                FlywheelConstants.KI,
                FlywheelConstants.KD,
                FlywheelConstants.ANGULAR_VELOCITY,
                FlywheelConstants.ANGULAR_ACCELERATION)
            .withFeedforward(
                new SimpleMotorFeedforward(
                    FlywheelConstants.KS, FlywheelConstants.KV, FlywheelConstants.KA))
            .withSimClosedLoopController(
                FlywheelConstants.SIM_KP,
                FlywheelConstants.SIM_KI,
                FlywheelConstants.SIM_KD,
                FlywheelConstants.ANGULAR_VELOCITY,
                FlywheelConstants.ANGULAR_ACCELERATION)
            .withSimFeedforward(
                new SimpleMotorFeedforward(
                    FlywheelConstants.SIM_KS, FlywheelConstants.SIM_KV, FlywheelConstants.SIM_KA))
            .withGearing(FlywheelConstants.GEARING)
            .withMotorInverted(false)
            .withIdleMode(MotorMode.COAST)
            .withStatorCurrentLimit(FlywheelConstants.CURRENT_LIMIT)
            .withClosedLoopRampRate(FlywheelConstants.RAMP_RATE)
            .withFollowers(
                Pair.of(followerSparkMax_RIGHT, false),
                Pair.of(followerSparkMax_LEFT_1, true),
                Pair.of(followerSparkMax_LEFT_2, true));

    SmartMotorController smartMotor =
        new SparkWrapper(sparkFlex, DCMotor.getNEO(1), flywheelMotorConfig);

    FlyWheelConfig flywheelConfig =
        new FlyWheelConfig(smartMotor)
            .withDiameter(FlywheelConstants.FLYWHEEL_DIAMETER)
            .withMass(FlywheelConstants.FLYWHEEL_MASS)
            .withSoftLimit(
                FlywheelConstants.FLYWHEEL_RPM_LIMIT_LOWER,
                FlywheelConstants.FLYWHEEL_RPM_LIMIT_UPPER);

    this.flywheel = new FlyWheel(flywheelConfig);
    this.motor = flywheel.getMotor();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs, AngularVelocity flywheelSetpoint) {
    inputs.supplyCurrentAmps = motor.getSupplyCurrent().map(c -> c.in(Amps)).orElse(0.0);
    inputs.statorCurrentAmps = motor.getStatorCurrent().in(Amps);
    inputs.appliedVolts = motor.getVoltage().in(Volts);
    inputs.temperatureCelsius = motor.getTemperature().in(Celsius);
    inputs.mechanismVelocityRPM = flywheel.getSpeed().in(RPM);
    inputs.targetVelocityRPM = flywheelSetpoint.in(RPM);
  }

  @Override
  public void stop() {
    flywheel.setDutyCycleSetpoint(0);
  }

  @Override
  public Command sysId() {
    return flywheel.sysId(
        FlywheelConstants.SYSID_STEP_VOLTAGE,
        FlywheelConstants.SYSID_RAMP_RATE,
        FlywheelConstants.SYSID_TEST_DURATION);
  }

  @Override
  public Command set(double dutyCycle) {
    return flywheel.set(dutyCycle);
  }

  @Override
  public Command setVelocity(AngularVelocity velocity) {
    return flywheel.setSpeed(velocity);
  }

  @Override
  public Command setVelocity(Supplier<AngularVelocity> velocity) {
    return flywheel.setSpeed(velocity);
  }

  @Override
  public void simIterate() {
    flywheel.simIterate();
  }
}
