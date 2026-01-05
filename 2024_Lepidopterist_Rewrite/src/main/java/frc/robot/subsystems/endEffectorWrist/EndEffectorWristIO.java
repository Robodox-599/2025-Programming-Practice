// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endEffectorWrist;

//interface
public class EndEffectorWristIO {
    protected double positionRad = 0.0;
    protected double velocityRadPerSec = 0.0;
    protected double wantedPosition = 0;
    protected boolean isWristInPosition = false;

    protected boolean isCoralDetectedInRamps = false;
    protected boolean isCoralDetectedInEndEffector = false;
    protected boolean isCoralFullyInEndEffector = false;

    public void updateInputs(){}

    public void stop(){}

    public void setPosition(double position){}
    public void setVelocity(double velocity){}

}
