package org.team157.robot.subsystems.intake;

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

public class IntakeIOSparkMax implements IntakeIO {

  private final FlyWheel intake;
  private final SmartMotorController motor;

  public IntakeIOSparkMax(SubsystemBase subsystem) {
    // TODO: id properly
    SparkMax sparkmax = new SparkMax(157, MotorType.kBrushless);
    SparkMax followerTalonfx = new SparkMax(158, MotorType.kBrushless);

    SmartMotorControllerConfig intakeRollerMotorConfig =
        new SmartMotorControllerConfig(subsystem)
            .withControlMode(ControlMode.OPEN_LOOP)
            .withTelemetry("IntakeRollerMotor", TelemetryConstants.TELEMETRY_VERBOSITY)
            .withMotorInverted(true)
            .withIdleMode(MotorMode.COAST)
            // TODO: make intake constants and put real values in here
            //   .withStatorCurrentLimit(IntakeConstants.CURRENT_LIMIT)
            //   .withGearing(IntakeConstants.GEARING)
            .withFollowers(Pair.of(followerTalonfx, false));

    SmartMotorController smartRollerMotor =
        new SparkWrapper(sparkmax, DCMotor.getNEO(1), intakeRollerMotorConfig);

    FlyWheelConfig intakeRollerConfig =
        new FlyWheelConfig(smartRollerMotor)
            .withTelemetry("Intake", TelemetryConstants.TELEMETRY_VERBOSITY)
            .withMass(Kilograms.of(0.5))
            .withDiameter(Inches.of(2));

    this.intake = new FlyWheel(intakeRollerConfig);
    this.motor = intake.getMotor();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.supplyCurrentAmps = motor.getSupplyCurrent().map(c -> c.in(Amps)).orElse(0.0);
    inputs.statorCurrentAmps = motor.getStatorCurrent().in(Amps);
    inputs.appliedVolts = motor.getVoltage().in(Volts);
    inputs.temperatureCelsius = motor.getTemperature().in(Celsius);
    inputs.mechanismVelocityDegreesPerSecond = motor.getMechanismVelocity().in(DegreesPerSecond);
    inputs.intakeRunning = intake.gte(DegreesPerSecond.of(5)).getAsBoolean();
  }

  @Override
  public Command stop() {
    return intake.set(0);
  }

  @Override
  public Command set(double dutyCycle) {
    return intake.set(dutyCycle);
  }

  @Override
  public void simIterate() {
    intake.simIterate();
  }
}
