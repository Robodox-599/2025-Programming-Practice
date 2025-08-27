package frc.robot.subsystems.intake.rollers;

public class Rollers {
  private final RollersIO io;
  private TargetState targetState = TargetState.STOPPED;
  private CurrentState currentState = CurrentState.STOPPED;
  private CurrentState previousState = CurrentState.STOPPED;

  public Rollers(RollersIO io) {
    this.io = io;
  }

  public enum TargetState{
    INTAKING,
    STOPPED
  }

  public enum CurrentState{
    INTAKING,
    STOPPED
  }

  public void updateInputs() {
    io.updateInputs();
  }

  private void stateTransitions() {
    previousState = currentState;
    switch (targetState) {
      case INTAKING:
        currentState = CurrentState.INTAKING;
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
        case INTAKING:
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
}