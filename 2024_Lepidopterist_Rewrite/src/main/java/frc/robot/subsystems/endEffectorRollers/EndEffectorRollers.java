package frc.robot.subsystems.endEffectorRollers;

import dev.doglog.DogLog;

public class EndEffectorRollers extends EndEffectorRollersIO{
    private final EndEffectorRollersIO io;
    private WantedState wantedState = WantedState.STOPPED;
    private CurrentState currentState = CurrentState.STOPPED;

    public enum WantedState{
        INTAKING,
        HOLD_CORAL,
        SCORING,
        ALGAE_INTAKING,
        ALGAE_HOLDING,
        ALGAE_SCORING,
        STOPPED
      }
    
      public enum CurrentState{
        INTAKING,
        HOLD_CORAL,
        SCORING,
        ALGAE_INTAKING,
        ALGAE_HOLDING,
        ALGAE_SCORING,
        STOPPED
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
    // suggested by starting with handleStateTransitions state machines first before applyStates stateMachine
    private void handleStateTransitions(){
      switch(wantedState){
        case INTAKING:
          currentState = CurrentState.INTAKING;
          break;
        case SCORING:
          currentState = CurrentState.SCORING;
          break;
        case HOLD_CORAL:
          currentState = CurrentState.HOLD_CORAL;
          break;
        case ALGAE_INTAKING:
          currentState = CurrentState.ALGAE_INTAKING;
          break;
        case ALGAE_HOLDING:
            currentState = CurrentState.ALGAE_HOLDING;
          break;
        case ALGAE_SCORING:
          currentState = CurrentState.ALGAE_SCORING;
          break;
        case STOPPED:
          currentState = CurrentState.STOPPED;
          break;
        
      }
    }

    private void applyStates() {
        switch (currentState) {
          case INTAKING:
            setVelocity(-0.5);
            break;
          case SCORING:
            setVelocity(-0.5);
            break;
          case HOLD_CORAL:
            stop();
            break;
          case ALGAE_INTAKING:
            setVelocity(0.5);
            break;
          case ALGAE_HOLDING:
            holdAlgae();
            break;
          case ALGAE_SCORING:
            setVelocity(-0.5);
            break;
          case STOPPED:
            stop();
            break;
          default:
            stop();
            break;
          }
        }

    public boolean isCoralDetected() {
        return io.isCoralDetected;
      }

    public boolean isAlgaeDetected(){
      return io.isAlgaeDetected;
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

    public void setWantedState(WantedState wantedState){
      this.wantedState = wantedState;
    }
}
