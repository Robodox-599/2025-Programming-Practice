package frc.robot.subsystems.endefectorwrist;

public class EndefectorWristConstants {
    public static final int endefectorWristMotorID = 15;
    public static final String endefectorWristCANBus = "rio";
    public static final int cancoderID = 17;
    public static final double gearRatio = 35;

    public static final double kP = 45;
    public static final double kI = 0;
    public static final double kD = 6;
    public static final double kV = 0.124 * gearRatio;
    public static final double kS = 0.085;
    public static final double kG = 0.415;

    public static final double supplyCurrentLimit = 40;
    public static final double cancoderMagnetOffset = -0.13720703125;
    public static final double discontinuityPoint = 0.4;
    public static final double wristTolerance = 0.02;

    public static final double maxVelocity = (12-kG-kS)/kV;
    public static final double maxAcceleration = maxVelocity/0.5;
}
