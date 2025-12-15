package frc.robot.subsystems.endefectorwrist;

import dev.doglog.DogLog;

public class EndefectorWrist {
 private final EndefectorWristIO io;
 private WantedState wantedState = WantedState.STOPPED;
 private CurrentState currentState = CurrentState.STOPPED;
 

 public enum WantedState {
  STOPPED,
  INTAKING_CORAL,
  INTAKING_ALGAE,
  SCORE_CORAL,
  SCORE_ALGAE,
 }

 public enum CurrentState {
  STOPPED,
  INTAKING_CORAL,
  INTAKING_ALGAE,
  SCORE_CORAL,
  SCORE_ALGAE,
 }

  public EndefectorWrist(EndefectorWristIO io) {
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
    case SCORE_CORAL:
      currentState = CurrentState.SCORE_CORAL;
    break;
    case INTAKING_ALGAE:
    currentState = CurrentState.INTAKING_ALGAE;
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
        setPosition(-0.3);
        break;
        case SCORE_CORAL:
        setPosition(-0.14);
        break;//prepare = -.21
      case INTAKING_ALGAE:
        setPosition(-0.1);
        break;
      case SCORE_ALGAE:
         setPosition(-0.21);
         break;
      default:
        stop();
        break;}
      }

      public void updateInputs(){
        io.updateInputs();
        handleStateTransitions();
        applyState();
        DogLog.log("EndefectorWrist/WantedState", wantedState.toString());
        DogLog.log("EndefectorWrist/CurrentState", currentState.toString());
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

      public boolean isWristInPosition(){
        return io.isWristInPosition;
      }
}
