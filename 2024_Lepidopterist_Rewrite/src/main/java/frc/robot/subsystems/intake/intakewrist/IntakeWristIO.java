// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.intakewrist;

import frc.robot.subsystems.intake.intakewrist.IntakeWristConstants.IntakeWristStates;

public abstract class IntakeWristIO {
  protected double tempCelsius = 0.0;
  protected double currentAmps = 0.0;
  protected double appliedVolts = 0.0;
  protected double velocity = 0.0;
  protected double targetPosition = 0.0;
  protected double currentPositionDegrees = 0.0;
  protected boolean atSetpoint = false;
  protected IntakeWristConstants.IntakeWristStates state = IntakeWristStates.STOW;


  public void updateInputs() {}

  public void setVoltage(double voltage) {}

  public void goToPose(double position) {}

  public void stop() {}

  public void setBrake(boolean brake) {}

  public void setAngle(IntakeWristConstants.IntakeWristStates state) {}

  public double getCurrentVolts() {
    return appliedVolts;
  }
}