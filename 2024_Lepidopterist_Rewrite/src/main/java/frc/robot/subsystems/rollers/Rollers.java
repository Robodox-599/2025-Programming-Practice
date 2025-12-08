package frc.robot.subsystems.rollers;
import dev.doglog.DogLog;
import frc.robot.subsystems.rollers.endefectorrollers.EndefectorRollers;
import frc.robot.subsystems.rollers.ramprollers.RampRollers;

public class Rollers {
 private final EndefectorRollers endefectorRollers;
 private final RampRollers rampRollers;
 private wantedSuperState wantedState = wantedSuperState.STOPPED;
 private currentSuperState currentState = currentSuperState.STOPPED;
 

    public enum wantedSuperState {
        STOPPED,
        RAMP_INTAKING,
        RAMP_HOLD_CORAL,
        ROLLERS_INTAKING,
        ENDEFECTOR_HOLD_CORAL,
        ENDEFECTOR_SCORE
    }

    public enum currentSuperState{
        STOPPED,
        RAMP_INTAKING,
        RAMP_HOLD_CORAL,
        ROLLERS_INTAKING,
        ENDEFECTOR_HOLD_CORAL,
        ENDEFECTOR_SCORE
    }

    public Rollers (EndefectorRollers endefectorRollers, RampRollers rampRollers) {
        this.endefectorRollers = endefectorRollers;
        this.rampRollers = rampRollers;
    }

    private void handleStateTransitions(){
        switch (wantedState) {
        case STOPPED:
        currentState = currentSuperState.STOPPED;
        break;
        case RAMP_INTAKING:
        if (rampRollers.isCoralDetected()) {
            wantedState = wantedSuperState.RAMP_HOLD_CORAL;
            currentState = currentSuperState.RAMP_HOLD_CORAL;
           } else {
             currentState = currentSuperState.RAMP_INTAKING;
           }
        break;
        case RAMP_HOLD_CORAL:
        if (!rampRollers.isCoralDetected()) {
            wantedState = wantedSuperState.RAMP_INTAKING;
            currentState = currentSuperState.RAMP_INTAKING;
           } else {
            currentState = currentSuperState.RAMP_HOLD_CORAL;
           }
        break;
        case ROLLERS_INTAKING:
        if (!rampRollers.isCoralDetected() && !endefectorRollers.isCoralInEndefector()) {
            wantedState = wantedSuperState.RAMP_INTAKING;
            currentState = currentSuperState.RAMP_INTAKING;
           } else if(!rampRollers.isCoralDetected() && endefectorRollers.isCoralInEndefector()){
            wantedState = wantedSuperState.ENDEFECTOR_HOLD_CORAL;
            currentState = currentSuperState.ENDEFECTOR_HOLD_CORAL;
           } else {
            currentState = currentSuperState.ROLLERS_INTAKING;
           }
        break;
        case ENDEFECTOR_HOLD_CORAL:
        if (!endefectorRollers.isCoralInEndefector()) {
            wantedState = wantedSuperState.RAMP_INTAKING;
            currentState = currentSuperState.RAMP_INTAKING;
           } else {
            currentState = currentSuperState.ENDEFECTOR_HOLD_CORAL;
           }
        break;
        case ENDEFECTOR_SCORE:
        if (!endefectorRollers.isCoralInEndefector()) {
            wantedState = wantedSuperState.RAMP_INTAKING;
            currentState = currentSuperState.RAMP_INTAKING;
           } else {
            currentState = currentSuperState.ENDEFECTOR_SCORE;
           }
        break;
        default:
        currentState = currentSuperState.STOPPED;
        break;}
        }

        private void applyState(){
            switch (currentState) {
            case STOPPED:
            rampRollers.setWantedState(RampRollers.WantedState.STOPPED);
            endefectorRollers.setWantedState(EndefectorRollers.WantedState.STOPPED);
            break;
            case RAMP_INTAKING:
            rampRollers.setWantedState(RampRollers.WantedState.INTAKING);
            endefectorRollers.setWantedState(EndefectorRollers.WantedState.STOPPED);
            break;
            case RAMP_HOLD_CORAL:
            rampRollers.setWantedState(RampRollers.WantedState.HOLD_CORAL);
            endefectorRollers.setWantedState(EndefectorRollers.WantedState.STOPPED);
            break;
            case ROLLERS_INTAKING:
            rampRollers.setWantedState(RampRollers.WantedState.TRANSFERING);
            endefectorRollers.setWantedState(EndefectorRollers.WantedState.INTAKING);
            break;
            case ENDEFECTOR_HOLD_CORAL:
            rampRollers.setWantedState(RampRollers.WantedState.STOPPED);
            endefectorRollers.setWantedState(EndefectorRollers.WantedState.HOLD_CORAL);
            break;
            case ENDEFECTOR_SCORE:
            rampRollers.setWantedState(RampRollers.WantedState.STOPPED);
            endefectorRollers.setWantedState(EndefectorRollers.WantedState.SCORE);
            break;
            default:
            rampRollers.setWantedState(RampRollers.WantedState.STOPPED);
            endefectorRollers.setWantedState(EndefectorRollers.WantedState.STOPPED);
            break;}
      }

        public void updateInputs(){
        handleStateTransitions();
        applyState();
        DogLog.log("Rollers/WantedState", wantedState);
        DogLog.log("Rollers/CurrentState", currentState);
      }


        public void setWantedState(wantedSuperState wantedState) {
            this.wantedState = wantedState;
          }
    }
