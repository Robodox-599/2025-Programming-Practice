package frc.robot.subsystems.endEffectorRollers;

public class EndEffectorRollersConstants {

    // Hardware
    public static final int EndEffectorRollersMotorID = 16;
    public static final String EndEffectorRollersCANBus = "rio";

    // Sensor
    public static final int beamBreakPort = 2;
    public static final double endEffectorRollersDebounceTimeSeconds = 0.2;

    // Gear Ratio
    public static final int endEffectorRollersGearRatio = 1;

    // Motor Control Constants (needs tuning)
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.0;

    // Feet forward 
    // tells the motor controller how much voltage to apply
    // before any error happens, based on how fast you want the motor to move
    public static final double kV = 0.001 * endEffectorRollersGearRatio;

    // Current Limits
    public static final double supplyCurrentLimit = 50;
    public static final double statorCurrentLimit = 50;
}
