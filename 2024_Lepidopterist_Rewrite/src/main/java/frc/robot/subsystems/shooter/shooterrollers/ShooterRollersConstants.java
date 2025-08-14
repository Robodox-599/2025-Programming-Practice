package frc.robot.subsystems.shooter.shooterrollers;

public class ShooterRollersConstants {
  public static enum ShooterRollersStates {
    INTAKING(0),
    SCORING(1),
    STOPPED(2);

    private final int index;

    ShooterRollersStates(int index) {
      this.index = index;
    }

    public int getIndex() {
      return index;
    }
  }

  public static final double[] rollersVelocities = {
    0.6, // intaking note
    -0.4, // scoring note
    0.0, // stopped
  };

  // Real layer constants

  public static final int rollersMotorID = 16;
  public static final String rollersMotorCANBus = "rio";

  public static final boolean EnableCurrentLimit = true;
  public static final int ContinousCurrentLimit = 50;
  public static final int PeakCurrentLimit = 50;
  public static final double PeakCurrentDuration = 0.1;

  public static final double gearRatio = 1.5;
  public static final double indexerMOI = 0.04;

  public static final double realP = 0.0;
  public static final double realI = 0.0;
  public static final double realD = 0.0;
  public static final double realS = 0.0;
  public static final double realV = 0.0;

  public static final double indexerDutyCycleOutHoldAlgae = 0.2;

  public static final int beakBreakPort = 3;

  public static final double beamBreakDebounce = 0.3;
  public static final double ensureCoralDebounce = 0.8;
  public static final double algaeDebounce = 0.5;

  public static final double rotationsToMoveAfterDetectingNote = 0.0;

  // Sim layer constants

  public static final double simkP = 6.7;
  public static final double simkI = 0.0;
  public static final double simkD = 0.0;
  public static final double simkS = 0.0;
  public static final double simkV = 0.0;
}