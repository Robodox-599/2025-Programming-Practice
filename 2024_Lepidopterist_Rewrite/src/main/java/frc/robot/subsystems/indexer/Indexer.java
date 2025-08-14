package frc.robot.subsystems.indexer;

import dev.doglog.DogLog;
import frc.robot.SafetyChecker;
//import frc.robot.SafetyChecker;
import frc.robot.subsystems.indexer.IndexerConstants.IndexerStates;
import frc.robot.util.Tracer;

public class Indexer {
  private final IndexerIO io;
  private final SafetyChecker safetyChecker;
  private WantedState wantedState = WantedState.STOPPED;
  private CurrentState currentState = CurrentState.STOPPED;
  private CurrentState previousState = CurrentState.STOPPED;

  public Indexer(IndexerIO io) {
    this.io = io;
    this.safetyChecker = new SafetyChecker();
  }

  public enum WantedState {
    INTAKING,
    ENSURING_NOTE,
    SCORING,
    HOLDING,
    STOPPED,
  }

  public enum CurrentState {
    INTAKING,
    ENSURING_NOTE,
    SCORING,
    HOLDING,
    STOPPED,
  }

  public void updateInputs() {
    Tracer.traceFunc("UpdateIO", io::updateInputs);
    Tracer.traceFunc("HandleStateTransitions", this::handleStateTransitions);
    Tracer.traceFunc("ApplyStates", this::applyStates);
    DogLog.log("Indexer/CurrentState", currentState);
    DogLog.log("Indexer/WantedState", wantedState);
  }

  private void handleStateTransitions() {
    previousState = currentState;
    switch (wantedState) {
      case INTAKING:
        currentState = CurrentState.INTAKING;
        break;
      case ENSURING_NOTE:
        currentState = CurrentState.ENSURING_NOTE;
        break;
      case SCORING:
        currentState = CurrentState.SCORING;
        break;
      case HOLDING:
        currentState = CurrentState.HOLDING;
        break;
      case STOPPED:
        currentState = CurrentState.STOPPED;
        break;
      default:
        currentState = CurrentState.STOPPED;
        break;
    }
  }

  private void applyStates() {
    if (previousState != currentState) {
      switch (currentState) {
        case INTAKING:
          setVelocity(IndexerStates.INTAKING);
          break;
        case ENSURING_NOTE:
          stop();
          break;
        case SCORING:
          setVelocity(IndexerStates.SCORING);
          break;
        case HOLDING:
          stop();
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

  public void setVelocity(IndexerStates state) {
    io.setVelocity(state);
  }

  public void stop() {
    io.stop();
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  public boolean isNoteDetected() {
    return io.isNoteDetected;
  }

  public boolean isNoteEnsured() {
    return io.isNoteEnsured;
  }
}