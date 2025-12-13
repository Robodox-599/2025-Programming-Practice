package frc.robot.subsystems.endEffectorRollers;

import dev.doglog.DogLog;

public class EndEffectorRollers extends EndEffectorRollersIO{
    private final EndEffectorRollersIO io;
    private WantedState wantedState = WantedState.STOPPED;
    private CurrentState currentState = CurrentState.STOPPED;

    public enum WantedState{
        INTAKING_CORAL,
        HOLDING_CORAL,
        SCORING_CORAL,
        INTAKING_ALGAE,
        HOLDING_ALGAE,
        SCORING_ALGAE,
        STOPPED
      }
    
      public enum CurrentState{
        INTAKING_CORAL,
        HOLDING_CORAL,
        SCORING_CORAL,
        INTAKING_ALGAE,
        HOLDING_ALGAE,
        SCORING_ALGAE,
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
        DogLog.log("endEffectorRollers/wantedState", wantedState);
        DogLog.log("endEffectorRollers/currentState", currentState);
    }

    // decides what state the mahcine should be in
    // suggested by starting with handleStateTransitions state machines first before applyStates stateMachine
    private void handleStateTransitions(){
      switch(wantedState){
        case INTAKING_CORAL:
          currentState = CurrentState.INTAKING_CORAL;
          break;
        case SCORING_CORAL:
          currentState = CurrentState.SCORING_CORAL;
          break;
        case HOLDING_CORAL:
          currentState = CurrentState.HOLDING_CORAL;
          break;
        case INTAKING_ALGAE:
          currentState = CurrentState.INTAKING_ALGAE;
          break;
        case HOLDING_ALGAE:
            currentState = CurrentState.HOLDING_ALGAE;
          break;
        case SCORING_ALGAE:
          currentState = CurrentState.SCORING_ALGAE;
          break;
        case STOPPED:
          currentState = CurrentState.STOPPED;
          break;
        default:
        currentState = CurrentState.STOPPED;
          break;
        
      }
    }

    private void applyStates() {
        switch (currentState) {
          case INTAKING_CORAL:
            setVelocity(0.15);
            break;
          case SCORING_CORAL:
            setVelocity(0.15);
            break;
          case HOLDING_CORAL:
            setPosition(io.holdCoralPosition);
            break;
          case INTAKING_ALGAE:
            setVelocity(-0.15);
            break;
          case HOLDING_ALGAE:
            holdAlgae();
            break;
          case SCORING_ALGAE:
            setVelocity(0.8);
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

    public boolean isAlgaeIntaked(){
      return io.isAlgaeIntaked;
    }

    public boolean isAlgaeScored(){
      return io.isAlgaeScored;
    }

    public void stop(){
        io.stop();
    }

    public void setVelocity(double velocity){
        io.setVelocity(velocity);
    }

    //of motor
    public void setPosition(double position){
        io.setPosition(position);
    }

    public void setWantedState(WantedState wantedState){
      this.wantedState = wantedState;
    }

    public void setEndeffectorHoldCoralPosition(){
      io.holdCoralPosition = io.getPosition();
    }

    // function that sets wantedCoralPosition to the current position of the ee rollers
}
