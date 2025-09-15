package frc.robot.subsystems.indexer;

import dev.doglog.DogLog;

public class Indexer {
  private final IndexerIO io;
  private TargetState targetState = TargetState.STOPPED;
  private CurrentState currentState = CurrentState.STOPPED;
  private CurrentState previousState = CurrentState.STOPPED;

  public Indexer(IndexerIO io) {
    this.io = io;
    DogLog.log("Rollers/CurrentState", currentState);
    DogLog.log("Rollers/WantedState", targetState);
  }

  public enum TargetState{
    NO_NOTE,
    INDEXING,
    NOTE_IN_POSTION,
    HOLD_NOTE,
    STOPPED
  }

  public enum CurrentState{
    NO_NOTE,
    INDEXING,
    NOTE_IN_POSTION,
    HOLD_NOTE,
    STOPPED
  }

  public void updateInputs() {
    io.updateInputs();
  }

  private void stateTransitions() {
    previousState = currentState;
    switch (targetState) {
      case NO_NOTE:
        currentState = CurrentState.NO_NOTE;
        break;
      case INDEXING:
        currentState = CurrentState.INDEXING;
        if(noteInPosition()){
          currentState = CurrentState.NOTE_IN_POSTION;
        } else {
          currentState = CurrentState.INDEXING;
        }
        break;
      case NOTE_IN_POSTION: 
      if(noteInPosition()){
        currentState = CurrentState.NOTE_IN_POSTION;
      } else {
        currentState = CurrentState.INDEXING;
      }
        break;
      case HOLD_NOTE:
        currentState = CurrentState.HOLD_NOTE;
        break;
      case STOPPED:
        currentState = CurrentState.STOPPED;
      default:
        currentState = CurrentState.STOPPED;
        break;
    }
  }

  private void setStates() {
    if (previousState != currentState) {
      switch (currentState) {
        case NO_NOTE:
          break;
        case INDEXING:
          setVelocity(0);
          break;
        case NOTE_IN_POSTION:
          stop();
          break;
        case HOLD_NOTE:
          stop();
        case STOPPED:
          stop();
          break;
        default:
          stop();
          break;
      }
    }
  }

  public void setVelocity(double velocity) {
    io.setVelocity(velocity);
  }

  public void stop() {
    io.stop();
  }

  public boolean noteDetected() {
    return io.noteDetected;
  }

  public boolean noteInPosition() {
    return io.noteInPosition;
  }
}