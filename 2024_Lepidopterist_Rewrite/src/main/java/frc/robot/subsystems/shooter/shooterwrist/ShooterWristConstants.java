package frc.robot.subsystems.shooter.shooterwrist;

public class ShooterWristConstants {
  // motor info
  public static final int wristMotorID = 15;
  public static final int cancoderID = 17;
  public static final String wristMotorCANBus = "rio";
  public static final double gearRatio = 58.78;
  public static final double wristMOI = 0.04;
  public static final double wristPositionTolerance = 0.02; // rotations

  // current limit stuff
  public static final boolean EnableCurrentLimit = true;
  public static final int ContinousCurrentLimit = 50;
  public static final int PeakCurrentLimit = 50;
  public static final double PeakCurrentDuration = 0.1;

  public static final double realkP = 15.0;
  public static final double realkI = 0.0;
  public static final double realkD = 0.0;
  public static final double realkS = -0.275;
  public static final double realkV = 6.96078949;
  public static final double realkG = -0.39;

  // sim stuff
  public static final double simkP = 6.9;
  public static final double simkI = 0.5;
  public static final double simkD = 2.25;
  public static final double simkV = 0.0;
  public static final double simkS = 0.0;
  public static final double simVelocityConstant = 0.2;

  public static final double cancoderOffset = -0.399658203125;

  // setpoints
  public static final double wristMinAngle = 0.52;
  public static final double wristMaxAngle = 1.05;

  // wrist state stuff
  public static enum WristStates {
    INTAKING(0),
    SCORING(1),
    POSITION_PREPARED(2);

    private final int index;

    WristStates(int index) {
      this.index = index;
    }

    public int getIndex() {
      return index;
    }
  }

  public static final double[] setpoints = {
    0.664, // intaking
    0.79, // scoring
    0.79, // position prepared / dormant position
  };
}