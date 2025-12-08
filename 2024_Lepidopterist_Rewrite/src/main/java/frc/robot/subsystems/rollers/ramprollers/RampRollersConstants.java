package frc.robot.subsystems.rollers.ramprollers;

public class RampRollersConstants {
    public static final int rampRollersMotorID = 23;
    public static final String rampRollersCANBus = "rio";
    public static final int beamBreakPort = 1;
    public static final double rampRollersDebounceTimeSeconds = 0.2;
    public static final double rampRollersGearRatio = 2;

    public static final double kP = 0.45;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0.03;
    public static final double kV = 0.124 * rampRollersGearRatio;

    public static final double supplyCurrentLimit = 40;
}
