package frc.robot.subsystems.shooter.wrist;

import dev.doglog.DogLog;

public class ShooterWrist {
  private final ShooterWristIO io;
  private WantedState wantedState = WantedState.STOPPED;
  private CurrentState currentState = CurrentState.STOPPED;

  public ShooterWrist(ShooterWristIO io) {
    this.io = io;
  }

  public enum WantedState {
      SHOOTING_SPEAKER,
      SHOOTING_AMP,
      HOLDING,
      STOW,
      STOPPED;
  }

  public enum CurrentState {
    SHOOTING_SPEAKER,
    SHOOTING_AMP,
    HOLDING,
    STOW,
    STOPPED;
  }

  public void updateInputs() {
    DogLog.log("Wrist/CurrentState", currentState);
    DogLog.log("Wrist/WantedState", wantedState);
  }

  private void handleStateTransitions() {
    switch (wantedState) {
      case SHOOTING_SPEAKER:
        currentState = CurrentState.SHOOTING_SPEAKER;
        break;
    case SHOOTING_AMP:
        currentState = CurrentState.SHOOTING_AMP;
        break;
    case HOLDING:
        currentState = CurrentState.HOLDING;
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
      case SHOOTING_SPEAKER:
        goToAngle(0);
        break;
      case SHOOTING_AMP:
        goToAngle(0);
        break;
      case HOLDING:
        holdAngle(0);
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

  public void holdAngle(double angle) {
    io.holdAngle(angle);
  }

  public ShooterWristIO getIO() {
    return io;
  }
}