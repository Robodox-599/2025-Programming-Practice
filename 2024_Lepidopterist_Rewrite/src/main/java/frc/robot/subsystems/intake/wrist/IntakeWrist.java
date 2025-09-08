package frc.robot.subsystems.intake.wrist;

import dev.doglog.DogLog;

public class IntakeWrist {
  private final IntakeWristIO io;
  private WantedState wantedState = WantedState.STOPPED;
  private CurrentState currentState = CurrentState.STOPPED;

  public IntakeWrist(IntakeWristIO io) {
    this.io = io;
  }

  public enum WantedState {
      INTAKING,
      STOW,
      STOPPED;
  }

  public enum CurrentState {
    INTAKING,
    STOW,
    STOPPED;
  }

  public void updateInputs() {
    DogLog.log("Wrist/CurrentState", currentState);
    DogLog.log("Wrist/WantedState", wantedState);
  }

  private void handleStateTransitions() {
    switch (wantedState) {
      case INTAKING:
        currentState = CurrentState.INTAKING;
        break;
      case STOW:
        currentState = CurrentState.STOPPED;
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
    switch (currentState) {
      case INTAKING:
        goToAngle(0);
        break;
      case STOW:
        goToAngle(0);
      break;
      case STOPPED:
        stop();
        break;
      default:
        stop();
        break;
    }
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  public void stop() {
    io.stop();
  }

  public void goToAngle(double angle) {
    io.goToAngle(angle);
  }

  public IntakeWristIO getIO() {
    return io;
  }
}