// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot.subsystems.intakeDeploy;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  // The IO interface for interacting with the hood's motor.
  private IntakeIO io;

  // Inputs from the motor, encoder, and mechanism, to be updated periodically and logged.
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public void setIO(IntakeIO io) {
    this.io = io;
  }

  /** Creates a new Intake. */
  public Intake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Updates the inputs to be logged by AdvantageKit and writes them to the Logger
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }
}
