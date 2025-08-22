package frc.robot.subsystems.shooter.shooterwrist;

import frc.robot.subsystems.shooter.shooterwrist.ShooterWristConstants.ShooterWristStates;

public class ShooterWrist {
  private final ShooterWristIO io;
  private WantedState wantedState = WantedState.STOP;
  private CurrentState currentState = CurrentState.STOP;
  private CurrentState previousState = CurrentState.STOP;

  public ShooterWrist(ShooterWristIO io) {
    this.io = io;
  }

  public void periodic(){
    io.updateInputs();
  }

  public enum WantedState{
    INTAKING,
    SCORING,
    HOLDNOTE,
    STOP,
  }

  public enum CurrentState{
    INTAKING,
    SCORING,
    HOLDNOTE,
    STOP,
  }

  public void handleStateTransitions(){
    previousState = currentState;
    switch (wantedState) {
      case INTAKING:
        currentState = CurrentState.INTAKING;
        break;
      case SCORING:
        currentState = CurrentState.SCORING;
        break;
      case HOLDNOTE:
        currentState = CurrentState.HOLDNOTE;
        break;
      case STOP:
        currentState = CurrentState.STOP;
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
          setAngle(ShooterWristStates.INTAKING);
          break;
        case SCORING:
          setAngle(ShooterWristStates.SCORING);
          break;
        case HOLDNOTE:
          setAngle(ShooterWristStates.HOLDNOTE);
          break;
        case STOP:
          setAngle(ShooterWristStates.STOP);
          break;
        default:
          setAngle(ShooterWristStates.STOP);
          break;
      }
    }
  }

  public void setAngle(ShooterWristConstants.ShooterWristStates state) {
    io.setAngle(state);
  }

  public boolean isAtSetpoint() {
    return io.atSetpoint;
  }
  
  public void stop() {
    io.stop();
  }
}