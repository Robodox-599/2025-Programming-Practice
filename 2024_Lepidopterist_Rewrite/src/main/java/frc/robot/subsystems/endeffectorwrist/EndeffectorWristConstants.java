package frc.robot.subsystems.endeffectorwrist;

public class EndeffectorWristConstants {
    public static final int endefectorWristMotorID = 15;
    public static final String endefectorWristCANBus = "rio";
    public static final int cancoderID = 17;
    public static final double gearRatio = 35;

    public static final double kP = 45;
    public static final double kI = 0;
    public static final double kD = 6;
    public static final double kS = 0;
    public static final double kV = 0.085;
    public static final double kG = 0.415;

    public static final double supplyCurrentLimit = 40;
    public static final double cancoderMagnetOffset = -0.13720703125;
    public static final double discontinuityPoint = 0.0;
    public static final double isWristInPositionTolerence = 0.2;
    public static final double MaxVelocity = 0.0;
    public static final double MaxAcceleration = 0.0;
}
