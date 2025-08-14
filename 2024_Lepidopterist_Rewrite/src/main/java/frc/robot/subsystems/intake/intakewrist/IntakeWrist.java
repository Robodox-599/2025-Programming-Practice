package frc.robot.subsystems.intake.intakewrist;

import dev.doglog.DogLog;
import frc.robot.SafetyChecker;
import frc.robot.subsystems.intake.intakewrist.IntakeWristConstants.WristStates;
import frc.robot.util.Tracer;

public class IntakeWrist {
  private final IntakeWristIO io;
  private final SafetyChecker safetyChecker;
  private WantedState wantedState = WantedState.STOPPED;
  private CurrentState currentState = CurrentState.STOPPED;
  private CurrentState previousState = CurrentState.STOPPED;

  public IntakeWrist(IntakeWristIO io, SafetyChecker safetyChecker) {
    this.io = io;
    this.safetyChecker = safetyChecker;
  }

  public enum WantedState {
    INTAKING,
    SCORING,
    POSITION_PREPARED,
    STOPPED,
  }

  public enum CurrentState {
    INTAKING,
    SCORING,
    POSITION_PREPARED,
    STOPPED,
  }

  public void updateInputs() {
    Tracer.traceFunc("UpdateIO", io::updateInputs);
    safetyChecker.setIntakeRunning(currentState != CurrentState.STOPPED);
    safetyChecker.updateIsAtSetpointIntake(isAtSetpoint());
    Tracer.traceFunc("HandleStateTransitions", this::handleStateTransitions);
    Tracer.traceFunc("ApplyStates", this::applyStates);
    DogLog.log("Wrist/CurrentState", currentState);
    DogLog.log("Wrist/WantedState", wantedState);
  }

  private void handleStateTransitions() {
    previousState = currentState;
    switch (wantedState) {
      case INTAKING:
        if (safetyChecker.isSafeIntake()) {
          currentState = CurrentState.INTAKING;
        }
        break;
      case POSITION_PREPARED:
        currentState = CurrentState.POSITION_PREPARED;
        break;
      case SCORING:
        currentState = CurrentState.SCORING;
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
          setAngle(WristStates.INTAKING);
          break;
        case POSITION_PREPARED:
          setAngle(WristStates.POSITION_PREPARED);
          break;
        case SCORING:
          setAngle(WristStates.SCORING);
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

  public void setAngle(WristStates state) {
    io.setAngle(state);
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  public boolean isAtSetpoint() {
    return io.atSetpoint;
  }

  public void stop() {
    io.stop();
  }

  public IntakeWristIO getIO() {
    return io;
  }
}