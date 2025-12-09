package frc.robot.subsystems.endefectorrollers;

import dev.doglog.DogLog;

public class EndefectorRollers {
 private final EndefectorRollersIO io;
 private WantedState wantedState = WantedState.STOPPED;
 private CurrentState currentState = CurrentState.STOPPED;

 public enum WantedState {
  STOPPED,
  INTAKING,
  HOLD_CORAL,
  SCORE
 }

 public enum CurrentState {
    STOPPED,
    INTAKING,
    HOLD_CORAL,
    SCORE
 }

  public EndefectorRollers(EndefectorRollersIO io) {
    this.io = io;
  }

  private void handleStateTransitions(){
    switch (wantedState) {
    case STOPPED:
    currentState = CurrentState.STOPPED;
    break;
    case INTAKING:
        currentState = CurrentState.INTAKING;
    break;
    case HOLD_CORAL:
      currentState = CurrentState.HOLD_CORAL;
    break;
    case SCORE:
      currentState = CurrentState.SCORE;
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
      case INTAKING:
        setVelocity(0.3); 
        break;
      case HOLD_CORAL:
        setPosition(io.holdPosition);
        break;
        case SCORE:
        setVelocity(0.5);

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
}
