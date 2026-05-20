package org.team157.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team157.robot.Constants.TelemetryConstants;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.local.SparkWrapper;

public class HopperIOSparkMax implements HopperIO {

  private final FlyWheel hopper;
  private final SmartMotorController motor;

  public HopperIOSparkMax(SubsystemBase subsystem) {
    // TODO: id properly
    SparkMax sparkmax = new SparkMax(157, MotorType.kBrushless);
    SparkMax followerTalonfx = new SparkMax(158, MotorType.kBrushless);

    SmartMotorControllerConfig hopperRollerMotorConfig =
        new SmartMotorControllerConfig(subsystem)
            .withControlMode(ControlMode.OPEN_LOOP)
            .withTelemetry("hopperRollerMotor", TelemetryConstants.TELEMETRY_VERBOSITY)
            .withMotorInverted(true)
            .withIdleMode(MotorMode.COAST)
            .withStatorCurrentLimit(HopperConstants.CURRENT_LIMIT)
            .withGearing(HopperConstants.GEARING)
            .withFollowers(Pair.of(followerTalonfx, false));

    SmartMotorController smartRollerMotor =
        new SparkWrapper(sparkmax, DCMotor.getNEO(1), hopperRollerMotorConfig);

    FlyWheelConfig hopperRollerConfig =
        new FlyWheelConfig(smartRollerMotor)
            .withTelemetry("hopper", TelemetryConstants.TELEMETRY_VERBOSITY)
            .withMass(Kilograms.of(0.5))
            .withDiameter(Inches.of(2));

    this.hopper = new FlyWheel(hopperRollerConfig);
    this.motor = hopper.getMotor();
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    inputs.supplyCurrentAmps = motor.getSupplyCurrent().map(c -> c.in(Amps)).orElse(0.0);
    inputs.statorCurrentAmps = motor.getStatorCurrent().in(Amps);
    inputs.appliedVolts = motor.getVoltage().in(Volts);
    inputs.temperatureCelsius = motor.getTemperature().in(Celsius);
    inputs.mechanismVelocityDegreesPerSecond = motor.getMechanismVelocity().in(DegreesPerSecond);
    inputs.hopperRunning = hopper.gte(DegreesPerSecond.of(5)).getAsBoolean();
  }

  @Override
  public Command stop() {
    return hopper.set(0);
  }

  @Override
  public Command set(double dutyCycle) {
    return hopper.set(dutyCycle);
  }

  @Override
  public void simIterate() {
    hopper.simIterate();
  }
}
