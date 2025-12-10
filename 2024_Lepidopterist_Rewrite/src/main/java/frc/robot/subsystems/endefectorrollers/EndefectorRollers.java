package frc.robot.subsystems.endefectorrollers;

import dev.doglog.DogLog;

public class EndefectorRollers {
 private final EndefectorRollersIO io;
 private WantedState wantedState = WantedState.STOPPED;
 private CurrentState currentState = CurrentState.STOPPED;

 public enum WantedState {
  STOPPED,
  INTAKING_CORAL,
  INTAKING_ALGAE,
  HOLD_CORAL,
  HOLD_ALGAE,
  SCORE_CORAL,
  SCORE_ALGAE,
 }

 public enum CurrentState {
  STOPPED,
  INTAKING_CORAL,
  INTAKING_ALGAE,
  HOLD_CORAL,
  HOLD_ALGAE,
  SCORE_CORAL,
  SCORE_ALGAE,
 }

  public EndefectorRollers(EndefectorRollersIO io) {
    this.io = io;
  }

  private void handleStateTransitions(){
    switch (wantedState) {
    case STOPPED:
    currentState = CurrentState.STOPPED;
    break;
    case INTAKING_CORAL:
        currentState = CurrentState.INTAKING_CORAL;
    break;
    case HOLD_CORAL:
      currentState = CurrentState.HOLD_CORAL;
    break;
    case SCORE_CORAL:
      currentState = CurrentState.SCORE_CORAL;
    break;
    case INTAKING_ALGAE:
    currentState = CurrentState.INTAKING_ALGAE;
    break;
    case HOLD_ALGAE:
    currentState = CurrentState.HOLD_ALGAE;
    break;
    case SCORE_ALGAE:
    currentState = CurrentState.SCORE_ALGAE;
    break;
    default:
    currentState = CurrentState.STOPPED;
    break;}
    }

    private void applyState(){
      switch (currentState) {
      case STOPPED:
        stop();
        break;
      case INTAKING_CORAL:
        setVelocity(0.3); 
        break;
      case HOLD_CORAL:
        setPosition(io.holdPosition);
        break;
        case SCORE_CORAL:
        setVelocity(0.5);
        break;
      case INTAKING_ALGAE:
        setVelocity(0.3); 
        break;
      case HOLD_ALGAE:
        holdAlgae();
        break;
      case SCORE_ALGAE:
        setVelocity(0.1);
        break;
      default:
        stop();
        break;}
      }

      public void updateInputs(){
        io.updateInputs();
        handleStateTransitions();
        applyState();
        DogLog.log("EndefectorRollers/WantedState", wantedState.toString());
        DogLog.log("EndefectorRollers/CurrentState", currentState.toString());
      }

      public void setVelocity(double velocity) {
        io.setVelocity(velocity);
      }

      public void holdAlgae(){
        io.holdAlgae();
      }

      public void stop() {
        io.stop();
      }

      public void setPosition(double position) {
        io.setPosition(position);
      }

      public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
      }

      public boolean isCoralInEndefector(){
        return io.isCoralInEndefector;
      }

      public boolean isAlgaeInEndefector() {
        return io.isAlgaeInEndefector;
    }
}
