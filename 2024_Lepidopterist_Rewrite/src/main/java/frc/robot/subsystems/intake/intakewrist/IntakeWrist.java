package frc.robot.subsystems.intake.intakewrist;

import frc.robot.subsystems.intake.intakewrist.IntakeWristConstants.IntakeWristStates;

public class IntakeWrist {
  private final IntakeWristIO io;
  private WantedState wantedState = WantedState.STOP;
  private CurrentState currentState = CurrentState.STOP;
  private CurrentState previousState = CurrentState.STOP;

  public IntakeWrist(IntakeWristIO io) {
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
          setAngle(IntakeWristStates.INTAKING);
          break;
        case SCORING:
          setAngle(IntakeWristStates.SCORING);
          break;
        case HOLDNOTE:
          setAngle(IntakeWristStates.HOLDNOTE);
          break;
        case STOP:
          setAngle(IntakeWristStates.STOP);
          break;
        default:
          setAngle(IntakeWristStates.STOP);
          break;
      }
    }
  }

  public void setAngle(IntakeWristConstants.IntakeWristStates state) {
    io.setAngle(state);
  }

  public boolean isAtSetpoint() {
    return io.atSetpoint;
  }
  
  public void stop() {
    io.stop();
  }
}