package frc.robot.subsystems.indexer;

import dev.doglog.DogLog;

public class Indexer {
  private final IndexerIO io;
  private TargetState targetState = TargetState.STOPPED;
  private CurrentState currentState = CurrentState.STOPPED;
  private CurrentState previousState = CurrentState.STOPPED;

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  public enum TargetState{
    NOTEDETECTED,
    STOPPED
  }

  public enum CurrentState{
    NOTEDETECTED,
    STOPPED
  }

  public void updateInputs() {
    io.updateInputs();
  }

  private void handleStateTransitions() {
    previousState = currentState;
    switch (targetState) {
      case NOTEDETECTED:
        if(noteDetected()){
          currentState = CurrentState.NOTEDETECTED;
        } else {
          currentState = CurrentState.STOPPED;
        }
        break;
      case STOPPED:
        currentState = CurrentState.STOPPED;
      default:
        currentState = CurrentState.STOPPED;
        break;
    }
  }

  private void applyStates() {
    if (previousState != currentState) {
      switch (currentState) {
        case NOTEDETECTED:
          setVelocity(0);
          break;
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

  public boolean noteEnsured() {
    return io.noteEnsured;
  }
}