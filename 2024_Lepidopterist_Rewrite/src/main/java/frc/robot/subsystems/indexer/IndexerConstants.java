package frc.robot.subsystems.indexer;

public class IndexerConstants {
  
  public static final int indexerMotorID = 0;
  public static final String indexerMotorCANBus = "rio";

  public static final boolean EnableCurrentLimit = true;
  public static final int ContinousCurrentLimit = 50;
  public static final int PeakCurrentLimit = 50;
  public static final double PeakCurrentDuration = 0.1;

  public static final double gearRatio = 0;
  public static final double indexerMOI = 0;

  public static final double simkP = 0;
  public static final double simkI = 0;
  public static final double simkD = 0;
  public static final double simkS = 0;
  public static final double simkV = 0;

  public static final double realP = 0;
  public static final double realI = 0;
  public static final double realD = 0;
  public static final double realS = 0;
  public static final double realV = 0;

  public static final int beakBreakPort = 0;
  public static final double noteDebounce = 0;
  public static final double indexerHoldNote = 0;

  public static enum indexerState {
    NOTEDETECTED(0),
    INDEXING(1),
    NOTEINPOSITION(2),
    NOTENOTDETECTED(3),
    STOPPED(4);
    
    private final int index;
    indexerState(int index) {
      this.index = index;
    }
}}
