package frc.robot.subsystems.rollers.endeffectorrollers;

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
    private void handleStateTransitions(){}

    // runs motor based on current state
    private void applyStates() {}

    public boolean isCoralDetected() {
        return io.isCoralDetectedEndEffectorRollers;
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
