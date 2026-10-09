package org.team157.robot.subsystems.intakeDeploy;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Represents the Intake Deploy subsystem, a rack and pinion that moves the intake from inside the
 * robot out past the frame perimeter to pick up balls, and wiggles it to agitate balls toward the
 * hopper and uptake.
 */
public class IntakeDeploy extends SubsystemBase {

  // The IO interface for interacting with the rack's motor and potentiometer.
  private IntakeDeployIO io;

  // Inputs from the motor, potentiometer, and mechanism, to be updated periodically and logged.
  private final IntakeDeployIOInputsAutoLogged inputs = new IntakeDeployIOInputsAutoLogged();

  /** Creates a new IntakeDeploy. */
  public IntakeDeploy() {}

  /**
   * Specifies the IO implementation to be used for the IntakeDeploy.
   *
   * @param io An implementation of the IntakeDeploy's IO layer, i.e. IntakeDeployIOSparkFlex
   */
  public void setIO(IntakeDeployIO io) {
    this.io = io;
  }

  /**
   * Moves the rack to a position, giving up after a timeout so a jam can't stall the motor.
   *
   * @param position Rack position to go to.
   * @param timeoutSeconds Maximum time to try to reach the position.
   */
  private Command moveTo(Distance position, double timeoutSeconds) {
    return io.runTo(position, IntakeDeployConstants.POSITION_TOLERANCE).withTimeout(timeoutSeconds);
  }

  /**
   * Deploys the intake out of the robot to its intaking position near the floor.
   *
   * @return a {@link Command} moving the rack to the deployed position.
   */
  public Command deploy() {
    return moveTo(
            IntakeDeployConstants.DEPLOYED_POSITION, IntakeDeployConstants.MOVE_TIMEOUT.in(Seconds))
        .withName("IntakeDeploy Deploy");
  }

  /**
   * Retracts the intake back into its starting position inside the robot.
   *
   * @return a {@link Command} moving the rack to the retracted position.
   */
  public Command retract() {
    return moveTo(
            IntakeDeployConstants.RETRACTED_POSITION,
            IntakeDeployConstants.MOVE_TIMEOUT.in(Seconds))
        .withName("IntakeDeploy Retract");
  }

  /**
   * Repeatedly moves the intake partway in and back out to agitate balls toward the hopper and
   * uptake. Runs until interrupted, intended to be bound with whileTrue. Each move has a short
   * timeout so a full hopper blocking the intake doesn't stop the wiggle.
   *
   * @return a {@link Command} moving the rack between the wiggle and deployed positions.
   */
  public Command wiggle() {
    double wiggleMoveTimeout = IntakeDeployConstants.WIGGLE_MOVE_TIMEOUT.in(Seconds);
    return Commands.sequence(
            moveTo(IntakeDeployConstants.WIGGLE_IN_POSITION, wiggleMoveTimeout),
            moveTo(IntakeDeployConstants.DEPLOYED_POSITION, wiggleMoveTimeout))
        .repeatedly()
        .withName("IntakeDeploy Wiggle");
  }

  /**
   * Resets the motor encoder to the potentiometer's position. Useful if the rack slipped teeth.
   *
   * @return a {@link Command} reseeding the encoder from the potentiometer.
   */
  public Command seedEncoderFromPot() {
    return Commands.runOnce(() -> io.seedEncoderFromPot()).ignoringDisable(true);
  }

  /**
   * Set the duty cycle output of the rack motor. Primarily used for manual control
   *
   * @param dutycycle The power to be applied to the motor.
   */
  public Command set(double dutycycle) {
    return io.set(dutycycle);
  }

  /**
   * Sets the default command of the intake deploy, stopping motor output when no other commands are
   * running. The motor is in brake mode, so the rack stays where it was left.
   *
   * @return Command setting the duty cycle output of the rack's motor to 0
   */
  public Command getDefault() {
    return io.stop();
  }

  /** Gets the rack position, used for posing the intake in the Mechanism3D. */
  public Distance getPosition() {
    return Inches.of(inputs.positionInches);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Updates the inputs to be logged by AdvantageKit and writes them to the Logger
    io.updateInputs(inputs);
    Logger.processInputs("IntakeDeploy", inputs);

    // While disabled the rack can be moved by hand, so keep the encoder in sync with the pot.
    if (DriverStation.isDisabled()
        && Math.abs(inputs.potPositionInches - inputs.positionInches)
            > IntakeDeployConstants.ENCODER_RESYNC_THRESHOLD.in(Inches)) {
      io.seedEncoderFromPot();
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation.
    io.simIterate();
  }
}
