// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endEffectorWrist;

//constnats accessible everywhere under endEffectorWrist
public class EndEffectorWristConstants {
    public static final int endEffectorWristMotorID = 0;
    public static final int endEffectorWristCANCoderID = 0;
    public static final int rampBeamBreakPort = 1;
    public static final int endEffectorBeamBreakPort = 0;
    public static final String endEffectorWristCANBus = "rio";

    //PID & feedforward
    public static final double kP = 0; //output per unit error in position
    public static final double kI = 0; //outpout per unit of integrated error in position
    public static final double kD = 0; //output per unit of error in vleocity
    public static final double kS = 0; //output needed to overcome static friction
    public static final double kG = 0; //output needed to overcome gravity

    //magic motion
    public static final double endEffectorWristMaxVelocity = 0;
    public static final double endEffectorWristMaxAcceleration = 0;

    //physical constants
    public static final double endEffectorWristGearRatio = 0;
    public static final double endEffectorWristMagnetOffset = 0;
    public static final double absoluteDiscontinuityPoint = 0;
    public static final double endEffectorWristpositionTollerance = 0;


}
