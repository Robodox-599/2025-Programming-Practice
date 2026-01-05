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
        STOPPED,
        PREPARED,
        HANDING_CORAL,
        SCORING_CORAL,
        INTAKING_ALGAE,
        SCORING_ALGAE
    }

    public enum CurrentState{
        STOPPED,
        PREPARED,
        HANDING_CORAL,
        SCORING_CORAL,
        INTAKING_ALGAE,
        SCORING_ALGAE
    }

    //creates a new EndEffectorWrist
    public EndEffectorWrist(EndEffectorWristIO io){
        this.io = io;
    }

    private void handleStateTransitions(){
        switch(wantedState){
            case PREPARED:
                currentState = CurrentState.PREPARED;
                break;
            case HANDING_CORAL:
                currentState = CurrentState.HANDING_CORAL;
                break;
            case SCORING_CORAL:
                currentState = CurrentState.SCORING_CORAL;
                break;
            case INTAKING_ALGAE:
                currentState = CurrentState.INTAKING_ALGAE;
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

    private void applyStates(){
        switch(currentState){
            case PREPARED:
                setPosition(-0.21);
                break;
            case HANDING_CORAL:
                setPosition(-0.3);
                break;
            case SCORING_CORAL:
                setPosition(-0.14);
                break;
            case INTAKING_ALGAE:
                setPosition(-0.1);
                break;
            case SCORING_ALGAE:
                setPosition(-0.21);
                break;
            case STOPPED:
                stop();
                break;
            default:
                stop();
                break;
        }
    }

    @Override
    public void updateInputs(){
        io.updateInputs();
        handleStateTransitions();
        applyStates();
        DogLog.log("EndEffectorWrist/WantedState", wantedState);
        DogLog.log("EndEffectorWrist/CurrentState", currentState);
    }

    public boolean isWristInPosition(){
        return io.isWristInPosition;
    }

    @Override
    public void stop(){
        io.stop();
    }

    @Override
    public void setPosition(double position){
        io.setPosition(position);
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

}