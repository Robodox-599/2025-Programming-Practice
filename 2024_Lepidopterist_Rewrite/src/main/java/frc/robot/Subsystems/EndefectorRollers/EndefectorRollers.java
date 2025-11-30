package frc.robot.Subsystems.EndefectorRollers;

public class EndefectorRollers {
 private final EndefectorRollersIO io;
 private WantedState wantedState = WantedState.STOPPED;
 private CurrentState currentState = CurrentState.STOPPED;

 public enum WantedState {
  STOPPED,
  INTAKING,
  HOLD_CORAL,
  SCORE_L1,
  SCORE_L2_L3,
  SCORE_L4
 }

 public enum CurrentState {
    STOPPED,
    INTAKING,
    HOLD_CORAL,
    SCORE_L1,
    SCORE_L2_L3,
    SCORE_L4
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
      if (isCoralInEndefector()) {
       wantedState = WantedState.HOLD_CORAL;
       currentState = CurrentState.HOLD_CORAL;
      } else {
        currentState = CurrentState.INTAKING;
      }
    break;
    case HOLD_CORAL:
    if (!isCoralInEndefector()) {
      wantedState = WantedState.INTAKING;
      currentState = CurrentState.INTAKING;
     } else {
      currentState = CurrentState.HOLD_CORAL;
     }
    break;
    case SCORE_L1:
    if (!isCoralInEndefector()) {
      wantedState = WantedState.INTAKING;
      currentState = CurrentState.INTAKING;
     } else {
      currentState = CurrentState.SCORE_L1;
     }
    break;
    case SCORE_L2_L3:
    if (!isCoralInEndefector()) {
      wantedState = WantedState.INTAKING;
      currentState = CurrentState.INTAKING;
     } else {
      currentState = CurrentState.SCORE_L2_L3;
     }
    break;
    case SCORE_L4:
    if (!isCoralInEndefector()) {
      wantedState = WantedState.INTAKING;
      currentState = CurrentState.INTAKING;
     } else {
      currentState = CurrentState.SCORE_L4;
     }
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
        setVelocity(0); 
        break;
      case HOLD_CORAL:
        setPosition(io.holdPosition);
        break;
        case SCORE_L1:
        setVelocity(-0);
        break;
        case SCORE_L2_L3:
        setVelocity(-0);
        break;
        case SCORE_L4:
        setVelocity(-0);
        break;
      default:
        stop();
        break;}
      }

      public void updateInputs(){
        handleStateTransitions();
        applyState();
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
