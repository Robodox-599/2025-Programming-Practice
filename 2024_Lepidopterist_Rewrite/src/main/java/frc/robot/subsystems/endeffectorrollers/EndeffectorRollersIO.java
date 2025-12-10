// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffectorrollers;


public abstract class EndeffectorRollersIO {
  protected double position = 0.0;
  protected double velocity = 0.0;
  protected double statorCurrent = 0.0;
  protected double supplyCurrent = 0.0;
  protected double appliedVolts = 0.0;
  protected double tempCelsius = 0.0;

  protected boolean isCoralDetected = false;
  protected boolean isAlgaeDetected = false;
  protected double heldCurrentPosition = 0.0;

  public void updateInputs() {}

  public void setVelocity(double velocity) {}

  public void setPosition(double position) {}

  public double heldCurrentPosition() {
    return heldCurrentPosition;
  }

  public void holdAlgae(double dutyCycle){}

  public void stop() {}
}
