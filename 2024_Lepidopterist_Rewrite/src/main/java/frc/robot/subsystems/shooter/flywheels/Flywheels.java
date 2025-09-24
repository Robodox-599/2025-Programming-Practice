package frc.robot.subsystems.shooter.flywheels;

import dev.doglog.DogLog;

public class Flywheels {
  private final FlywheelsIO io;
  private TargetState targetState = TargetState.STOPPED;
  private CurrentState currentState = CurrentState.STOPPED;
  private CurrentState previousState = CurrentState.STOPPED;

  public Flywheels(FlywheelsIO io) {
    this.io = io;
    DogLog.log("Shooter/Flywheels/CurrentState", currentState);
    DogLog.log("Shooter/Flywheels/WantedState", targetState);
  }

  public enum TargetState{
    SHOOTING,
    STOPPED
  }

  public enum CurrentState{
    SHOOTING,
    STOPPED
  }

  public void updateInputs() {
    io.updateInputs();
  }

  private void stateTransitions() {
    previousState = currentState;
    switch (targetState) {
      case SHOOTING:
        currentState = CurrentState.SHOOTING;
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
        case SHOOTING:
          setVelocity(0);
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

//   public boolean areFlywheelsAtSpeed() {
//     io.areFlywheelsAtSpeed();
//   }
}
