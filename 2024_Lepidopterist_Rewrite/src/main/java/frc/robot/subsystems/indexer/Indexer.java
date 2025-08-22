package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.indexer.IndexerConstants.IndexerStates;

public class Indexer {
  private final IndexerIO io;
  private Timer beamBreakTimer = new Timer(); 
  private DigitalInput beamBreak;
  private WantedState wantedState = WantedState.STOP;
  private CurrentState currentState = CurrentState.STOP;
  private CurrentState previousState = CurrentState.STOP;

  public Indexer(IndexerIO io) {
    this.io = io;
    beamBreakTimer.start();
    beamBreak = new DigitalInput(IndexerConstants.beamBreakPort);
  }

  public enum WantedState{
    INTAKING,
    SCORING,
    HOLDNOTE,
    STOP,
  }

  public enum CurrentState{
    INTAKING,
    SCORING,
    HOLDNOTE,
    STOP,
  }

  public void periodic() {
      io.updateInputs();

      if(beamBreak.get())
      {
        beamBreakTimer.reset();
      }
  }

  public void handleStateTransitions(){
    previousState = currentState;
    switch (wantedState) {
      case INTAKING:
        currentState = CurrentState.INTAKING;
        break;
      case SCORING:
        currentState = CurrentState.SCORING;
        break;
      case HOLDNOTE:
        currentState = CurrentState.HOLDNOTE;
        break;
      case STOP:
        currentState = CurrentState.STOP;
        break;
      default:
        currentState = CurrentState.STOP;
        break;
    }
  }

  public void applyStates(){
    if(previousState != currentState){
      switch (currentState) {
        case INTAKING:
          setVelocity(IndexerStates.INTAKING);
          break;
        case SCORING:
          setVelocity(IndexerStates.SCORING);
          break;
        case HOLDNOTE:
          setVelocity(IndexerStates.HOLDNOTE);
          break;
        case STOP:
          setVelocity(IndexerStates.STOP);
          break;
        default:
          setVelocity(IndexerStates.STOP);
          break;
      }
    }
  }

  public void setVelocity(IndexerStates state) {
    io.setVelocity(state);
  }

  public void stop() {
    io.stop();
  }

  public boolean isNoteDetected() {
    return io.isNoteDetected;
  }
}