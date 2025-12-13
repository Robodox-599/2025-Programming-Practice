// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wristRollers;

//constnats accessible everywhere under wristRollers
public class WristRollersConstants {
    public static final int wristRollersMotorID = 0;
    public static final int wristRollersCancoderID = 0;
    public static final String wristRollersCANbus = "rio";

    //PID & feedforward
    public static final double kS = 0; //output needed to overcome static friction
    public static final double kG = 0; //output needed to overcome gravity
    public static final double kP = 0; //output per unit error in position
    public static final double kI = 0; //outpout per unit of integrated error in position
    public static final double kD = 0; //output per unit of error in vleocity

    //magic motion
    public static final double wristRollersMaxVelocity = 0;
    public static final double wristRollersMaxAcceleration = 0;

    //physical constants
    public static final double wristRollersGearRatio = 0;
    public static final double wristRollersMagnetOffset = 0;
    public static final double wristRollerspositionTollerance = 0;


}
