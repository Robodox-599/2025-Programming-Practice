package frc.robot.subsystems.rollers;

import frc.robot.subsystems.rollers.endeffectorrollers.EndEffectorRollers;
import frc.robot.subsystems.rollers.endeffectorrollers.EndEffectorRollersIO;
import frc.robot.subsystems.rollers.ramprollers.RampRollers;
import frc.robot.subsystems.rollers.ramprollers.RampRollersIO;

public class Rollers {
    private final RampRollers rampRollers;
    private final EndEffectorRollers endEffectorRollers;
    private WantedSuperState wantedSuperState = WantedSuperState.STOPPED;
    private CurrentSuperState currentSuperState = CurrentSuperState.STOPPED;
    private CurrentSuperState endEffectorRampRollersCurrentState = CurrentSuperState.STOPPED;
    private final EndEffectorRollersIO endEffectorRollersIO;
    private final RampRollersIO rampRollersIO;

    public enum WantedSuperState {
        RAMP_INTAKING,
        RAMP_HOLDING_CORAL,
        ENDEFFECTOR_INTAKING,
        ENDEFFECTOR_HOLDING_CORAL,
        ENDEFFECTOR_SCORING_CORAL,
        STOPPED
    }

    public enum CurrentSuperState {
        RAMP_INTAKING,
        RAMP_HOLDING_CORAL,
        ENDEFFECTOR_INTAKING,
        ENDEFFECTOR_HOLDING_CORAL,
        ENDEFFECTOR_SCORING_CORAL,
        STOPPED
    }

    public Rollers(RampRollers rampRollers, EndEffectorRollers endEffectorRollers) {
        this.rampRollers = rampRollers;
        this.endEffectorRollers = endEffectorRollers;
    }

    public void updateInputs() {
        rampRollers.updateInputs();
        endEffectorRollers.updateInputs();
        handleStateTransitions();
        applyStates();
    }

    private void handleStateTransitions() {
        switch (wantedSuperState) {
            case STOPPED:
                currentSuperState = CurrentSuperState.STOPPED;
                rampRollers.setWantedState(RampRollers.WantedState.STOPPED);
                endEffectorRollers.setWantedState(EndEffectorRollers.WantedState.STOPPED);
                break;

            case RAMP_INTAKING:
                rampRollers.setWantedState(RampRollers.WantedState.INTAKING);
                currentSuperState = CurrentSuperState.RAMP_INTAKING;
                if (rampRollers.isCoralDetected()) {
                    wantedSuperState = WantedSuperState.RAMP_HOLDING_CORAL;
                }
                break;

            case RAMP_HOLDING_CORAL:
                rampRollers.setWantedState(RampRollers.WantedState.HOLD_CORAL);
                currentSuperState = CurrentSuperState.RAMP_HOLDING_CORAL;
                if (!rampRollers.isCoralDetected()) {
                    wantedSuperState = WantedSuperState.RAMP_INTAKING;
                }
                break;

            case ENDEFFECTOR_INTAKING:
                endEffectorRollers.setWantedState(EndEffectorRollers.WantedState.INTAKING);
                currentSuperState = CurrentSuperState.ENDEFFECTOR_INTAKING;
                if (endEffectorRollers.isCoralDetected()) {
                    wantedSuperState = WantedSuperState.ENDEFFECTOR_HOLDING_CORAL;
                }
                break;

            case ENDEFFECTOR_HOLDING_CORAL:
                endEffectorRollers.setWantedState(EndEffectorRollers.WantedState.HOLD_CORAL);
                currentSuperState = CurrentSuperState.ENDEFFECTOR_HOLDING_CORAL;
                // You can add a condition here if you want to transition to scoring
                break;

            case ENDEFFECTOR_SCORING_CORAL:
                endEffectorRollers.setWantedState(EndEffectorRollers.WantedState.SCORING);
                currentSuperState = CurrentSuperState.ENDEFFECTOR_SCORING_CORAL;
                if (!endEffectorRollers.isCoralDetected()) {
                    wantedSuperState = WantedSuperState.ENDEFFECTOR_INTAKING;
                }
                break;
        }
    }

    private void applyStates() {
        switch (endEffectorRampRollersCurrentState) {
            case STOPPED:
                endEffectorRollersStop();
                rampRollersStop();
                break;
            case ENDEFFECTOR_INTAKING:
                setEndEffectorRollersVelocity(-0.15);
                break;
            case RAMP_INTAKING:
                setRampRollersVelocity(-0.15);
            case ENDEFFECTOR_HOLDING_CORAL:
                setEndEffectorRollersPosition(endEffectorRollersIO.wantedCoralPositionEndEffectorRollers);
                break;
            case RAMP_HOLDING_CORAL:
                setRampRollersPosition(rampRollersIO.wantedCoralPositionRampRollers);
                break;
            case ENDEFFECTOR_SCORING_CORAL:
                setEndEffectorRollersVelocity(-0.15);
                break;
            default:
                endEffectorRollersStop();
                break;
        }

    }

    // Optional: helper methods to manually set the super state
    public void setWantedSuperState(WantedSuperState state) {
        this.wantedSuperState = state;
    }

    public CurrentSuperState getCurrentSuperState() {
        return this.currentSuperState;
    }

    public void endEffectorRollersStop(){
        endEffectorRollersIO.stop();
    }

    public void rampRollersStop(){
        rampRollersIO.stop();
    }

    public void setEndEffectorRollersVelocity(double velocity){
        endEffectorRollersIO.setVelocity(velocity);
    }

    public void setRampRollersVelocity(double velocity){
        rampRollersIO.setVelocity(velocity);
    }

    public void setEndEffectorRollersPosition(double position){
        endEffectorRollersIO.setPosition(position);
    }

    public void setRampRollersPosition(double position){
        rampRollersIO.setPosition(position);
    }


}
