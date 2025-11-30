// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector.endeffectorrollers;

public class EndeffectorRollersConstants {
  public static final int endeffectorRollersMotorID = 23;
    public static final String endeffectorRollersCANBus = "rio";
    public static final int beamBreakPort = 1;
    public static final double endeffectorRollersDebounceTimeSeconds = 0.2;
    public static final double endeffectorRollersGearRatio = 2;

    public static final double kP = 0.45;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kV = 0.124 * endeffectorRollersGearRatio;

    public static final double supplyCurrentLimit = 40;
}
