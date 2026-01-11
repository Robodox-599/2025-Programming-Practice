// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import dev.doglog.DogLog;
import frc.robot.subsystems.endEffectorRollers.EndEffectorRollers;
import frc.robot.subsystems.endEffectorWrist.EndEffectorWrist;
import frc.robot.subsystems.rampRollers.RampRollers;

/** Add your docs here. */
public class SuperStructure {
    private final EndEffectorRollers endEffectorRollers;
    private final RampRollers rampRollers;
    private final EndEffectorWrist endEffectorWrist;
    private WantedSuperState wantedSuperState = WantedSuperState.STOPPED;
    private CurrentSuperState currentSuperState = CurrentSuperState.STOPPED;

    public enum WantedSuperState{
        STOPPED,
        PREPARED,
        INTAKING_CORAL_TO_RAMP, // Coral enters funnel
        TRANSFERING_CORAL,
        INTAKING_ALGAE,
        SCORING_CORAL,
        SCORING_ALGAE
    }

    public enum CurrentSuperState{
        STOPPED,
        PREPARED,
        INTAKING_CORAL_RAMP,
        INTAKING_CORAL_TO_RAMP, // corral enters rampRollers
        TRANSFERING_CORAL,
        INTAKING_ALGAE,
        SCORING_CORAL,
        SCORING_ALGAE
    }

    public SuperStructure(RampRollers rampRollers, EndEffectorRollers endEffectorRollers, EndEffectorWrist endEffectorWrist){
        this.rampRollers = rampRollers;
        this.endEffectorRollers = endEffectorRollers;
        this.endEffectorWrist = endEffectorWrist;
    }

    public void updateInputs(){
        handleStateTransitions();
        applyStates();
        DogLog.log("SuperStructure/WantedSuperState", wantedSuperState);
        DogLog.log("SuperStructure/CurrentSuperState", currentSuperState);
    }
    
    private void handleStateTransitions(){
        switch(wantedSuperState){
            case STOPPED:
                currentSuperState = CurrentSuperState.STOPPED;
                break;
            case PREPARED:
                currentSuperState = CurrentSuperState.PREPARED;
                break;
            case INTAKING_CORAL_TO_RAMP:
                currentSuperState = CurrentSuperState.INTAKING_CORAL_RAMP;
                break;
            case TRANSFERING_CORAL:
                currentSuperState = CurrentSuperState.TRANSFERING_CORAL;
                break;
            case INTAKING_ALGAE:
                currentSuperState = CurrentSuperState.INTAKING_ALGAE;
                break;
            case SCORING_CORAL:
                currentSuperState = CurrentSuperState.SCORING_CORAL;
                break;
            case SCORING_ALGAE:
                currentSuperState = CurrentSuperState.SCORING_ALGAE;
                break;
            default:
                currentSuperState = CurrentSuperState.STOPPED;
                break;

        }
    }

    private void applyStates(){
        switch(currentSuperState){
            case STOPPED:
                rampRollers.setWantedState(RampRollers.WantedState.STOPPED);
                endEffectorRollers.setWantedState(EndEffectorRollers.WantedState.STOPPED);
                endEffectorWrist.setWantedState(EndEffectorWrist.WantedState.STOPPED);
                break;
            case PREPARED:
                endEffectorWrist.setWantedState(EndEffectorWrist.WantedState.PREPARED);
                rampRollers.setWantedState(RampRollers.WantedState.HOLD_CORAL);
                endEffectorRollers.setWantedState(EndEffectorRollers.WantedState.HOLDING_ALGAE); //Also holds coral if there isn't algae
                break;
            case INTAKING_CORAL_RAMP:
                rampRollers.setWantedState(RampRollers.WantedState.INTAKING);
                endEffectorWrist.setWantedState(EndEffectorWrist.WantedState.PREPARED);
                endEffectorRollers.setWantedState(EndEffectorRollers.WantedState.HOLDING_ALGAE);
                break;
            case TRANSFERING_CORAL:
                endEffectorWrist.setWantedState(EndEffectorWrist.WantedState.HANDING_CORAL);
                rampRollers.setWantedState(RampRollers.WantedState.TRANSFERING);
                endEffectorRollers.setWantedState(EndEffectorRollers.WantedState.INTAKING_CORAL);
                break;
            case INTAKING_ALGAE:
                endEffectorWrist.setWantedState(EndEffectorWrist.WantedState.INTAKING_ALGAE);
                endEffectorRollers.setWantedState(EndEffectorRollers.WantedState.INTAKING_ALGAE);
                rampRollers.setWantedState(RampRollers.WantedState.HOLD_CORAL);
                break;
            case SCORING_CORAL:
                endEffectorWrist.setWantedState(EndEffectorWrist.WantedState.SCORING_CORAL);
                endEffectorRollers.setWantedState(EndEffectorRollers.WantedState.SCORING_CORAL);
                rampRollers.setWantedState(RampRollers.WantedState.HOLD_CORAL);
                break;
            case SCORING_ALGAE:
                endEffectorWrist.setWantedState(EndEffectorWrist.WantedState.SCORING_ALGAE);
                endEffectorRollers.setWantedState(EndEffectorRollers.WantedState.SCORING_ALGAE);
                rampRollers.setWantedState(RampRollers.WantedState.HOLD_CORAL);
                break;
            default:
                rampRollers.setWantedState(RampRollers.WantedState.STOPPED);
                endEffectorRollers.setWantedState(EndEffectorRollers.WantedState.STOPPED);
                endEffectorWrist.setWantedState(EndEffectorWrist.WantedState.STOPPED);
                break;
        }

    }

    public void setWantedState(WantedSuperState wantedState){
        this.wantedSuperState = wantedState;
    }
}
