package org.team157.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Represents the Intake subsystem, which feeds balls from the hopper up into the flywheel for
 * shooting.
 */
public class Intake extends SubsystemBase {

  // The IO interface for interacting with the Intake's motor.
  private IntakeIO io;

  // Inputs from the motor and mechanism, to be updated periodically and logged.
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  /** Creates a new Intake. */
  public Intake() {}

  /**
   * Specifies the IO implementation to be used for the intake .
   *
   * @param io An implementation of the intake 's IO layer, i.e. IntakeIOTalonFX
   */
  public void setIO(IntakeIO io) {
    this.io = io;
  }

  /**
   * Sets the default command of the Intake, stopping motor output when no other commands are
   * running.
   *
   * @return Command setting the duty cycle output of the Intake's motor to 0
   */
  public Command setDefault() {
    return io.stop();
  }

  /**
   * Set the duty cycle of the Intake roller motors.
   *
   * @param dutyCycle The power to be applied to the motors, between -1 and 1.
   * @return {@link Command} setting the duty cycle of the Intake roller motors.
   */
  public Command set(double dutyCycle) {
    return io.set(dutyCycle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Updates the inputs to be logged by AdvantageKit and writes them to the Logger
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    io.simIterate();
  }
}
