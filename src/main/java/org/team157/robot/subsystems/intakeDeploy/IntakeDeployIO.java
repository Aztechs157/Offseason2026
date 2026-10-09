package org.team157.robot.subsystems.intakeDeploy;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.AutoLog;

/**
 * Defines the input data to be logged by AdvantageKit, along with methods and {@link Command}s
 * which an implementation of this IO interface must have.
 */
public interface IntakeDeployIO {

  /**
   * Represents the set of inputs which are to be logged by AdvantageKit and updated by an
   * implementation of the {@link IntakeDeployIO} interface.
   */
  @AutoLog
  public static class IntakeDeployIOInputs {
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double appliedVolts = 0.0;
    public double temperatureCelsius = 0.0;
    /** Rack position according to the motor's encoder, used for closed loop control */
    public double positionInches = 0.0;
    /** Rack position setpoint of the closed loop controller */
    public double targetPositionInches = 0.0;

    public double velocityInchesPerSecond = 0.0;
    /** Raw potentiometer reading, 0 to 1 (fraction of 5V) */
    public double potRaw = 0.0;
    /** Rack position according to the potentiometer */
    public double potPositionInches = 0.0;
  }

  /**
   * Updates the inputs to be logged by AdvantageKit.
   *
   * @param inputs The set of inputs to be logged, including information on the motor, encoder, and
   *     mechanism.
   */
  default void updateInputs(IntakeDeployIOInputs inputs) {}

  /** Updates the values for the simulated version of the intake deploy mechanism. */
  default void simIterate() {}

  /**
   * Resets the motor's encoder to the position read by the potentiometer. Does nothing in
   * simulation, where there is no potentiometer.
   */
  default void seedEncoderFromPot() {}

  /**
   * Moves the rack to a position, ending once it is within tolerance. The motor keeps holding the
   * position after the command ends until another command takes over.
   *
   * @param position Rack position to go to
   * @param tolerance How close the rack must get for the command to end
   * @return a {@link Command} moving the rack to the specified position.
   */
  default Command runTo(Distance position, Distance tolerance) {
    return Commands.none();
  }

  /**
   * Stops the rack.
   *
   * @return a {@link Command} setting the motor's output power to 0.
   */
  default Command stop() {
    return Commands.none();
  }

  /**
   * Directly sets the output power of the rack's motor
   *
   * @param dutycycle Power to be applied to the motor, from 1 to -1.
   * @return a {@link Command} applying the specified power to the motor.
   */
  default Command set(double dutycycle) {
    return Commands.none();
  }
}
