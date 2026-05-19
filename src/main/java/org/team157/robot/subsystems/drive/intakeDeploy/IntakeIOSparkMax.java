package org.team157.robot.subsystems.drive.intakeDeploy;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class IntakeIOSparkMax implements IntakeIO {

  private final Elevator intake;
  private final SmartMotorController motor;

  public IntakeIOSparkMax(SubsystemBase subsystem, int canId) {
    SparkMax sparkmax = new SparkMax(canId, MotorType.kBrushless);

    // Step 1: Create SmartMotorControllerConfig
    SmartMotorControllerConfig smcConfig =
        new SmartMotorControllerConfig(subsystem)
            .withGearing(new MechanismGearing(GearBox.fromStages("5:1")))
            .withMechanismCircumference(Inches.of(19.6)) // sprocket circumference
            .withClosedLoopController(10, 0, 0.5)
            .withFeedforward(new ElevatorFeedforward(0.1, 0.2, 0.5, 0.01))
            .withTrapezoidalProfile(MetersPerSecond.of(1.0), MetersPerSecondPerSecond.of(2.0));

    // Step 2: Create SmartMotorController (TalonFXWrapper)
    SmartMotorController smc = new SparkWrapper(sparkmax, DCMotor.getNEO(1), smcConfig);

    // Step 3: Create IntakeConfig with the SmartMotorController
    ElevatorConfig intakeConfig =
        new ElevatorConfig(smc)
            .withDrumRadius(Inches.of(0.7)) // Drum radius for pulley
            .withMass(Pounds.of(10)) // Carriage mass - used for simulation physics
            .withHardLimits(Meters.of(0), Meters.of(1.5)) // Physical hard stops for sim
            .withSoftLimits(Meters.of(0.02), Meters.of(1.2))
            .withStartingHeight(Meters.of(0))
            .withAngle(Degrees.of(0)) // Vertical elevator
            .withTelemetry("Intake", TelemetryVerbosity.HIGH);

    // Step 4: Create Intake mechanism - handles simulation automatically!
    this.intake = new Elevator(intakeConfig);

    // Get reference to underlying SmartMotorController for telemetry
    this.motor = intake.getMotor();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Pull telemetry data from the underlying SmartMotorController
    inputs.positionMeters = motor.getMeasurementPosition().in(Meters);
    inputs.velocityMetersPerSec = motor.getMeasurementVelocity().in(MetersPerSecond);
    inputs.appliedVolts = motor.getVoltage().in(Volts);
    inputs.supplyCurrentAmps = motor.getSupplyCurrent().map(c -> c.in(Amps)).orElse(0.0);
    inputs.statorCurrentAmps = motor.getStatorCurrent().in(Amps);
    inputs.temperatureCelsius = motor.getTemperature().in(Celsius);
    // inputs.targetPositionMeters =
    //     motor.getTargetPosition().map(d -> d.in(Meters)).orElse(0.0);
  }



  @Override
  public void setTargetPosition(double meters) {
    // Use SmartMotorController's setPosition method with Distance
    motor.setPosition(Meters.of(meters));
  }

  @Override
  public void stop() {
    motor.setVoltage(Volts.of(0));
  }

  /** Access the Elevator mechanism for command helpers like run() and runTo() */
  public Elevator getIntake() {
    return intake;
  }   
     @Override
    public void simIterate(){
        intake.simIterate();
    }
}
