// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.shooterwrist;

import frc.robot.subsystems.shooter.shooterwrist.ShooterWristConstants.ShooterWristStates;

public abstract class ShooterWristIO {
  protected double tempCelsius = 0.0;
  protected double currentAmps = 0.0;
  protected double appliedVolts = 0.0;
  protected double velocity = 0.0;
  protected double targetPosition = 0.0;
  protected double currentPositionDegrees = 0.0;
  protected boolean atSetpoint = false;
  protected ShooterWristConstants.ShooterWristStates state = ShooterWristStates.STOW;


  public void updateInputs() {}

  public void setVoltage(double voltage) {}

  public void goToPose(double position) {}

  public void stop() {}

  public void setBrake(boolean brake) {}

  public void setAngle(ShooterWristConstants.ShooterWristStates state) {}

  public double getCurrentVolts() {
    return appliedVolts;
  }
}