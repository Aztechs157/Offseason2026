package org.team157.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team157.robot.Constants.FieldConstants;
import org.team157.robot.subsystems.hood.HoodConstants;
import org.team157.robot.subsystems.vision.Vision;

/**
 * Represents the Flywheel subsystem, which spins up to launch balls at a calculated velocity
 * towards the target.
 */
public class Flywheel extends SubsystemBase {

  // The IO interface for interacting with the flywheel's motors.
  private FlywheelIO io;

  // Used for the distance to the target in shot calculations.
  private Vision vision;

  // Inputs from the motors and mechanism, to be updated periodically and logged.
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  // The speed the flywheel was last told to run at, used to check if it is up to speed.
  private AngularVelocity targetVelocity = RPM.of(0);

  // Results of the shot calculation, updated every loop.
  private AngularVelocity desiredVelocity = RPM.of(0);
  private Angle desiredHoodAngle = HoodConstants.UPPER_SOFT_LIMIT;

  /** Creates a new Flywheel. */
  public Flywheel() {}

  /**
   * Specifies the IO implementation to be used for the Flywheel.
   *
   * @param io An implementation of the Flywheel's IO layer, i.e. FlywheelIOSparkflex
   * @param vision The vision subsystem, used for the distance to the target.
   */
  public void setIO(FlywheelIO io, Vision vision) {
    this.io = io;
    this.vision = vision;
  }

  /////////////////////////
  /// FLYWHEEL COMMANDS ///
  /////////////////////////

  /**
   * Sets the default command of the flywheel, stopping motor output when no other commands are
   * running.
   *
   * @return Command setting the duty cycle output of the flywheel's motor to 0
   */
  public Command getDefault() {
    return io.set(0).beforeStarting(() -> targetVelocity = RPM.of(0));
  }

  /**
   * Set the duty cycle output of the flywheel motor.
   *
   * @param dutyCycle The power to be applied to the motor, between -1 and 1.
   * @return {@link Command} setting the duty cycle of the flywheel.
   */
  public Command set(double dutyCycle) {
    return io.set(dutyCycle).beforeStarting(() -> targetVelocity = RPM.of(0));
  }

  /**
   * Set the flywheel to a fixed target angular velocity.
   *
   * @param speed The target angular velocity.
   * @return {@link Command} setting the flywheel to the specified velocity.
   */
  public Command setVelocity(AngularVelocity speed) {
    return io.setVelocity(speed).beforeStarting(() -> targetVelocity = speed);
  }

  /**
   * Set the flywheel to the velocity calculated for the current distance and height to the target.
   *
   * @return {@link Command} continuously updating the flywheel velocity.
   */
  public Command setDynamicVelocity() {
    return io.setVelocity(
        () -> {
          targetVelocity = desiredVelocity;
          return desiredVelocity;
        });
  }

  /**
   * Runs SysId on the flywheel to measure its feedforward constants. Runs the dynamic and
   * quasistatic tests forward and in reverse, so make sure no balls are loaded.
   *
   * @return {@link Command} running the flywheel SysId tests.
   */
  public Command sysId() {
    return io.sysId();
  }

  ////////////////////////
  /// FLYWHEEL METHODS ///
  ////////////////////////

  /**
   * Gets the current velocity of the flywheel.
   *
   * @return The current angular velocity of the flywheel.
   */
  public AngularVelocity getVelocity() {
    return RPM.of(inputs.mechanismVelocityRPM);
  }

  /**
   * Whether the flywheel is spinning and within {@link FlywheelConstants#VELOCITY_TOLERANCE} of the
   * speed it was last told to run at.
   *
   * @return true if the flywheel is up to speed.
   */
  public boolean isAtTargetVelocity() {
    return targetVelocity.gt(RPM.of(0))
        && getVelocity().isNear(targetVelocity, FlywheelConstants.VELOCITY_TOLERANCE);
  }

  ////////////////////////////////////////////
  /////// DYNAMIC VELOCITY CALCULATION ///////
  ////////////////////////////////////////////

  /**
   * Calculates required ball velocity (m/s) for given distance, height, and launch angle <br>
   * Using projectile motion equations: y = x*tan(θ) - (g*x²)/(2*v₀²*cos²(θ)) <br>
   * Solving for v₀: v₀ = sqrt((g*x²)/(2*cos²(θ)*(x*tan(θ) - y)))
   *
   * @param distance The horizontal distance to the target in meters.
   * @param height The vertical height of the target in meters.
   * @param theta The launch angle in radians.
   * @return The required ball velocity in meters per second.
   */
  static double velocityFunction(double distance, double height, double theta) {
    double g = 9.81;
    double heightDifference = height - FlywheelConstants.HEIGHT.in(Meters);
    double denominator =
        2 * Math.cos(theta) * Math.cos(theta) * (distance * Math.tan(theta) - heightDifference);
    if (denominator <= 0) {
      return 157; // Invalid shot parameters, return arbitrary velocity
    }
    return Math.sqrt((g * distance * distance) / denominator);
  }

  /**
   * Calculates the flywheel velocity and hood angle for the current shot. Searches the hood's range
   * for the launch angle that needs the slowest ball, since slower shots are more consistent. Under
   * a trench the hood is held at {@link HoodConstants#TRENCH_SAFE_ANGLE} so it clears, and only the
   * flywheel speed is calculated. Assumes the hood angle is the angle the ball leaves at, measured
   * up from horizontal. TODO: confirm against the real hood, and offset the angle if they differ.
   *
   * @param height Target height in meters.
   * @param distance Distance from the shooter to the target in meters.
   * @param isUnderTrench Whether the hood is under a trench.
   */
  public void setShotParams(double height, double distance, boolean isUnderTrench) {
    double lowerBound = HoodConstants.LOWER_SOFT_LIMIT.in(Radians);
    double upperBound = HoodConstants.UPPER_SOFT_LIMIT.in(Radians);
    if (isUnderTrench) {
      lowerBound = HoodConstants.TRENCH_SAFE_ANGLE.in(Radians);
      upperBound = HoodConstants.TRENCH_SAFE_ANGLE.in(Radians);
    }
    double steps = 50;
    double stepSize = (upperBound - lowerBound) / steps;

    double theta = lowerBound;
    double ballVelocity = velocityFunction(distance, height, lowerBound);

    for (int i = 1; i <= steps; i++) {
      double x = lowerBound + i * stepSize;
      double y = velocityFunction(distance, height, x);
      if (y < ballVelocity) {
        ballVelocity = y;
        theta = x;
      }
    }

    // Convert ball velocity (m/s) to flywheel RPM:
    // flywheelRPM = (ballVelocity * 60) / (π * flywheel_diameter)
    // multiplied by SPEED_FACTOR to account for air resistance and wheel slip
    double flywheelDiameterMeters = FlywheelConstants.FLYWHEEL_DIAMETER.in(Meters);
    double desiredRPM =
        (ballVelocity * 60) / (Math.PI * flywheelDiameterMeters) * FlywheelConstants.SPEED_FACTOR;

    desiredVelocity =
        RPM.of(
            Math.min(
                Math.max(desiredRPM, FlywheelConstants.MIN_DYNAMIC_VELOCITY.in(RPM)),
                FlywheelConstants.MAX_DYNAMIC_VELOCITY.in(RPM)));
    desiredHoodAngle = Radians.of(theta);

    Logger.recordOutput("Flywheel/Shot/BallVelocityMetersPerSecond", ballVelocity);
    Logger.recordOutput("Flywheel/Shot/DesiredVelocity", desiredVelocity);
    Logger.recordOutput("Flywheel/Shot/DesiredHoodAngle", desiredHoodAngle);
  }

  /**
   * Gets the flywheel velocity calculated for the current shot.
   *
   * @return The desired angular velocity of the flywheel.
   */
  public AngularVelocity getDesiredVelocity() {
    return desiredVelocity;
  }

  /**
   * Gets the hood angle calculated for the current shot.
   *
   * @return The desired hood angle.
   */
  public Angle getDesiredHoodAngle() {
    return desiredHoodAngle;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Recalculate the shot for the robot's current position
    setShotParams(
        FieldConstants.positionDetails.getTargetHeight(),
        vision.getDistanceToTarget(),
        vision.isUnderTrench());
    // Updates the inputs to be logged by AdvantageKit and writes them to the Logger
    io.updateInputs(inputs, targetVelocity);
    Logger.processInputs("Flywheel", inputs);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    io.simIterate();
  }
}
