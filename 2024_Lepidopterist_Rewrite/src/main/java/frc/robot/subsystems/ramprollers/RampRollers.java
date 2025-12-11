
package frc.robot.subsystems.ramprollers;

import dev.doglog.DogLog;

public class RampRollers {
 private final RampRollersIO io;
 private WantedState wantedState = WantedState.STOPPED;
 private CurrentState currentState = CurrentState.STOPPED;

 public enum WantedState {
  STOPPED,
  INTAKING,
  HOLD_CORAL,
  TRANSFERING
 }

 public enum CurrentState {
  STOPPED,
  INTAKING,
  HOLD_CORAL,
  TRANSFERING
 }

  public RampRollers(RampRollersIO io) {
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
    case TRANSFERING:
      currentState = CurrentState.TRANSFERING;
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
        setVelocity(-0.2); 
        break;
      case HOLD_CORAL:
        setPosition(io.holdPosition);
        break;
        case TRANSFERING:
        setVelocity(-0.2);
        break;
      default:
        stop();
        break;}
      }

      public void updateInputs(){
        io.updateInputs();
        handleStateTransitions();
        applyState();
        DogLog.log("RampRollers/WantedState", wantedState.toString());
        DogLog.log("RampRollers/CurrentState", currentState.toString());
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

      public boolean isCoralDetected(){
        return io.isCoralDetected;
      }
}
