// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endEffectorWrist;

//constnats accessible everywhere under endEffectorWrist
public class EndEffectorWristConstants {
    //physical constants
    public static final double endEffectorWristGearRatio = 35;
    public static final double endEffectorWristMagnetOffset = -0.13720703125;
    public static final double absoluteDiscontinuityPoint = 0.4;
    public static final double endEffectorWristpositionTollerance = 0.02;

    //iDs
    public static final int endEffectorWristMotorID = 15;
    public static final int endEffectorWristCANCoderID = 17;
    public static final int rampBeamBreakPort = 1;
    public static final int endEffectorBeamBreakPort = 0;
    public static final String endEffectorWristCANBus = "rio";

    //PID & feedforward
    public static final double kP = 45; //output per unit error in position
    public static final double kI = 0; //outpout per unit of integrated error in position
    public static final double kD = 0; //output per unit of error in vleocity
    public static final double kS = 0.085; //output needed to overcome static friction
    public static final double kG = 0.415; //output needed to overcome gravity
    public static final double kV = 0.124 * endEffectorWristGearRatio;

    //magic motion
    public static final double endEffectorWristMaxVelocity = ((12 - kG - kS) / kV);
    public static final double endEffectorWristMaxAcceleration = endEffectorWristMaxVelocity / 0.5;


}
