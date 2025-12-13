// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wristRollers;

//interface
public class WristRollersIO {
    protected double positionRad = 0.0;
    protected double velocityRadPerSec = 0.0;

    public void updateInputs(){}

    public void setPosition(double Position){}
    public void setVelocity(double velocity){}

    public void stop(){}

}
