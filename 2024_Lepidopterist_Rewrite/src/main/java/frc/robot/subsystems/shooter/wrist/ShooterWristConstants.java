package frc.robot.subsystems.shooter.wrist;

public class ShooterWristConstants {

    public static final int wristMotorID = 0;
    public static final String wristMotorCANBus = "rio";
    public static final double gearRatio = 0;
    public static final double wristMOI = 0;
    public static final int cancoderID = 0;
  
    public static final boolean EnableCurrentLimit = true;
    public static final int ContinousCurrentLimit = 50;
    public static final int PeakCurrentLimit = 50;
    public static final double PeakCurrentDuration = 0.1;

    public static final double simkP = 0;
    public static final double simkI = 0;
    public static final double simkD = 0;
    public static final double simkV = 0;
    public static final double simkS = 0;
    public static final double simVelocityConstant = 0;
    
    public static final double realkP = 0;
    public static final double realkI = 0;
    public static final double realkD = 0;
    public static final double realkS = 0;
    public static final double realkV = 0;
    public static final double realkG = 0;
  
    public static final double cancoderOffset = 0;

    public static enum WristStates {
      SHOOTING_SPEAKER(0),
      SHOOTING_AMP(1),
      HOLDING(2),
      STOW(3),
      STOPPED(4);

      private final int index;
      WristStates(int index) {
        this.index = index;
      }
      public int getIndex() {
        return index;
      }
    }
  }