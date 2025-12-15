// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endEffectorWrist;

import dev.doglog.DogLog;

//the state machine
public class EndEffectorWrist extends EndEffectorWristIO {
    private final EndEffectorWristIO io;
    private WantedState wantedState = WantedState.STOPPED;
    private CurrentState currentState = CurrentState.STOPPED;
    
    public enum WantedState{
        PREPARED,
        HANDING_CORAL,
        SCORING_CORAL,
        INTAKING_GROUND_ALGAE,
        INTAKING_REEF_ALGAE,
        SCORING_ALGAE,
        STOPPED
    }

    public enum CurrentState{
        PREPARED,
        HANDING_CORAL,
        SCORING_CORAL,
        INTAKING_GROUND_ALGAE,
        INTAKING_REEF_ALGAE,
        SCORING_NET_ALGAE,
        SCORING_PROCESSOR_ALGAE,
        SCORING_ALGAE,
        STOPPED
    }

    //creates a new EndEffectorWrist
    public EndEffectorWrist(EndEffectorWristIO io){
        this.io = io;
    }

    public void updateInputs(){
        io.updateInputs();
        handleStateTransitions();
        applyStates();
        DogLog.log("endEffectorWrist/wantedState", wantedState);
        DogLog.log("endEffectorWrist/currentState", currentState);
    }

    public void handleStateTransitions(){
        switch(wantedState){
            case PREPARED:
                currentState = CurrentState.PREPARED;
                break;
            case HANDING_CORAL:
                //to prevent wrist from retracting while the coral is still in the ramp roller
                currentState = CurrentState.HANDING_CORAL;
                break;
            case SCORING_CORAL:
                currentState = CurrentState.SCORING_CORAL;
                break;
            case INTAKING_GROUND_ALGAE:
                currentState = CurrentState.INTAKING_GROUND_ALGAE;
                break;
            case INTAKING_REEF_ALGAE:
                currentState = CurrentState.INTAKING_REEF_ALGAE;
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

    public void applyStates(){
        switch(currentState){
            case PREPARED:
                setPosition(0);
                break;
            case HANDING_CORAL:
                setPosition(0);
                break;
            case SCORING_CORAL:
                setPosition(0);
                break;
            case INTAKING_GROUND_ALGAE:
                setPosition(0);
                break;
            case INTAKING_REEF_ALGAE:
                setPosition(0);
                break;
            case SCORING_ALGAE:
                setPosition(0);
                break;
            case STOPPED:
                stop();
                break;
            default:
                stop();
                break;
        }
    }

    public void stop(){
        io.stop();
    }

    //to prevent wrist from retracting while the coral is still in the ramp roller
    public boolean isTransferComplete() {
        boolean wristHasCoral = io.isCoralDetectedInEndEffector;
        boolean rampHasNotCoral = !io.isCoralDetectedInRamps;

        return wristHasCoral && rampHasNotCoral;
    }

    public void setPosition(double position){
        io.setPosition(position);
    }

    public void setWantedState(WantedState stopped) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setWantedState'");
    }
}