package org.team157.robot.subsystems.drive.intakeDeploy;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double positionMeters = 0.0;
    public double velocityMetersPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;
    public double targetPositionMeters = 0.0;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void setTargetPosition(double meters) {}

  default void stop() {}

  default void simIterate() {}
}
