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
    NOTEDETECTED,
    INDEXING,
    NOTEINPOSTION,
    NOTENOTDETECTED,
    STOPPED
  }

  public enum CurrentState{
    NOTEDETECTED,
    INDEXING,
    NOTEINPOSTION,
    NOTENOTDETECTED,
    STOPPED
  }

  public void updateInputs() {
    io.updateInputs();
  }

  private void stateTransitions() {
    previousState = currentState;
    switch (targetState) {
      case NOTEDETECTED:
        if(noteDetected()){
          currentState = CurrentState.NOTEDETECTED;
        } else {
          currentState = CurrentState.NOTENOTDETECTED;
        }
        break;
      case INDEXING:
        currentState = CurrentState.INDEXING;
        if(noteInPosition()){
          currentState = CurrentState.NOTEINPOSTION;
        } else {
          currentState = CurrentState.INDEXING;
        }
        break;
      case NOTEINPOSTION: 
        currentState = CurrentState.NOTEINPOSTION;
        break;
      case NOTENOTDETECTED:
        currentState = CurrentState.NOTENOTDETECTED;
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
        case NOTEDETECTED:
          break;
        case INDEXING:
          setVelocity(0);
          break;
        case NOTEINPOSTION:
          stop();
          break;
        case NOTENOTDETECTED:
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