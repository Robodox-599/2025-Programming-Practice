package frc.robot.subsystems.endEffectorRollers;

import dev.doglog.DogLog;

public class EndEffectorRollers {
    private final EndEffectorRollersIO io;
    private WantedState wantedState = WantedState.STOPPED;
    private CurrentState currentState = CurrentState.STOPPED;

    public enum WantedState{
        STOPPED,
        INTAKING,
        HOLD_CORAL,
        SCORING,
      }
    
      public enum CurrentState{
        STOPPED,
        INTAKING,
        HOLD_CORAL,
        SCORING,
      }

    // Creates a new EndEffectorRollers
    public EndEffectorRollers(EndEffectorRollersIO io){
        this.io = io;
    }

    // Runs every .02 seconds
    public void updateInputs(){
        //update inputs first (reason)
        io.updateInputs();
        handleStateTransitions();
        applyStates();
        DogLog.log("EndEffectorRollers/wantedState", wantedState);
        DogLog.log("EndEffectorRollers/currentState", currentState);
    }

    // decides what state the mahcine should be in
    private void handleStateTransitions(){
      switch(wantedState){
        case STOPPED:
          currentState = CurrentState.STOPPED;
        case INTAKING:
          currentState = CurrentState.INTAKING;
        case SCORING:
          currentState = CurrentState.SCORING;
        case HOLD_CORAL:
          currentState = CurrentState.HOLD_CORAL;
      }
    }

    // runs motor based on current state
    private void applyStates() {
        switch (currentState) {
          case STOPPED:
            stop();
            break;
          case INTAKING:
            setVelocity(-0.5);
            break;
          case SCORING:
            setVelocity(-0.5);
            break;
          case HOLD_CORAL:
            setPosition(io.wantedCoralPosition);
          default:
            stop();
            break;
        }
      }

    public boolean isCoralDetected() {
        return io.isCoralDetected;
      }

    public void stop(){
        io.stop();
    }

    public void setVelocity(double velocity){
        io.setVelocity(velocity);
    }

    public void setPosition(double position){
        io.setPosition(position);
    }

    public Object setWantedState(WantedState intaking) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setWantedState'");
    }
}
