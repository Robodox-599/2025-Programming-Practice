package frc.robot.subsystems.intake.intakerollers;

public class IntakeRollerConstants {
  public static enum IntakeRollerStates {
    INTAKING(0),
    SCORING(1),      
    HOLDING(2),
    STOW(3);

    private final int index;

    IntakeRollerStates(int index) {
      this.index = index;
    }

    public int getIndex() {
      return index;
    }
  }

  public static final double[] intakeRollerVelocities = {
    0.6, // intaking note
    -0.4, // scoring note
    0.0, // holding note
    0.0, // stowed
  };

  // Real Motor config constants 

  public static final int rollersMotorID = 0;
  public static final String rollersMotorCANBus = "rio";

  public static final boolean EnableCurrentLimit = true;
  public static final int ContinousCurrentLimit = 50;
  public static final int PeakCurrentLimit = 50;
  public static final double PeakCurrentDuration = 0.1;

  public static final double gearRatio = 1.5;
  public static final double indexerMOI = 0.04;

  // PID constants

  public static final double realP = 0.0;
  public static final double realI = 0.0;
  public static final double realD = 0.0;
  public static final double realS = 0.0;
  public static final double realV = 0.0;
  
  // BeamBreak constants

  public static final int beamBreakPort = 3;
  public static final double beamBreakDebounce = 0.3;

  // Velocity Clamps

  public static final double indexerLowerLimit = 0.0;
  public static final double indexerUpperLimit = 1.0;

  // Sim PID

  public static final double simkP = 6.7;
  public static final double simkI = 0.0;
  public static final double simkD = 0.0;
  public static final double simkS = 0.0;
  public static final double simkV = 0.0;
}