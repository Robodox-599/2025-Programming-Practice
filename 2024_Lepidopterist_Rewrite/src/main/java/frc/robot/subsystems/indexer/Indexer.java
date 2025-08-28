package frc.robot.subsystems.indexer;

import frc.robot.subsystems.indexer.IndexerConstants.IndexerStates;

public class Indexer {
  private final IndexerIO io;
  private WantedState wantedState = WantedState.STOP;
  private CurrentState currentState = CurrentState.STOP;
  private CurrentState previousState = CurrentState.STOP;

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  public enum WantedState{
    INTAKING,
    SCORING,
    HOLD_NOTE,
    STOP,
    NO_NOTE,
  }

  public enum CurrentState{
    INTAKING,
    SCORING,
    HOLD_NOTE,
    STOP,
    NO_NOTE,
  }

  public void updateInputs() {
      io.updateInputs();
  }

  public void handleStateTransitions(){
    previousState = currentState;
    switch (wantedState) {
      case INTAKING:
        if(isNoteDetected()){
          currentState = CurrentState.HOLD_NOTE;
        }
        else{
          currentState = CurrentState.INTAKING;
        }
        break;
      case SCORING:
        if(!isNoteDetected()){
          currentState = CurrentState.NO_NOTE;
        }
        else{
          currentState = CurrentState.SCORING;
        }
        break;
      case HOLD_NOTE:
        currentState = CurrentState.HOLD_NOTE;
        break;
      case STOP:
        currentState = CurrentState.STOP;
        break;
      case NO_NOTE:
        if(!isNoteDetected()){
          currentState = CurrentState.NO_NOTE;
        }
        else{
          currentState = CurrentState.HOLD_NOTE;
        }
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
        case HOLD_NOTE:
          setVelocity(IndexerStates.HOLD_NOTE);
          break;
        case NO_NOTE:
          setVelocity(IndexerStates.NO_NOTE);
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