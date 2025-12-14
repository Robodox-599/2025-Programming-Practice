// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffectorwrist;

public abstract class EndeffectorWristIO {
  protected double statorCurrent = 0.0;
  protected double supplyCurrent = 0.0;
  protected double appliedVolts = 0.0;
  protected double tempCelsius = 0.0;
  protected double currentPosition = 0;
  protected double velocity = 0;

  protected double targetPosition = 0;
  protected boolean isWristInPosition = false;
  
  public void updateInputs() {}

  public void setPosition(double angle) {}
  
  public void stop() {}
}
