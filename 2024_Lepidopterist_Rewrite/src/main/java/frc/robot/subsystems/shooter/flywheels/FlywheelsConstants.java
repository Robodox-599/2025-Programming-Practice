package frc.robot.subsystems.shooter.flywheels;

public class FlywheelsConstants {
  
  public static final int topMotorID = 0;
  public static final String topMotorCANBus = "rio";
  public static final int bottomMotorID = 0;
  public static final String bottomMotorCANBus = "rio";

  public static final boolean EnableCurrentLimit = true;
  public static final int ContinousCurrentLimit = 50;
  public static final int PeakCurrentLimit = 50;
  public static final double PeakCurrentDuration = 0.1;

  public static final double gearRatio = 0;
  public static final double flywheelsMOI = 0;

  public static final double topSimkP = 0;
  public static final double topSimkI = 0;
  public static final double topSimkD = 0;
  public static final double topSimkS = 0;
  public static final double topsimkV = 0;

  public static final double bottomSimkP = 0;
  public static final double bottomSimkI = 0;
  public static final double bottomSimkD = 0;
  public static final double bottomSimkS = 0;
  public static final double bottomsimkV = 0;

  public static final double topRealP = 0;
  public static final double topRealI = 0;
  public static final double topRealD = 0;
  public static final double topRealS = 0;
  public static final double topRealV = 0;

  public static final double bottomRealP = 0;
  public static final double bottomRealI = 0;
  public static final double bottomRealD = 0;
  public static final double bottomRealS = 0;
  public static final double bottomRealV = 0;

  public static enum flywheelsState {
    SHOOTING(0),
    STOPPED(1);

    private final int index;
    flywheelsState(int index) {
      this.index = index;
    }
 }
}
