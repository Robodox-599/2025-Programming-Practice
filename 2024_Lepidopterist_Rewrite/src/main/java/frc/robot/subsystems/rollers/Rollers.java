// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.endeffectorrollers.EndeffectorRollers;
import frc.robot.subsystems.ramprollers.RampRollers;

public class Rollers extends SubsystemBase {
  
  private final RampRollers rampRollers;
  private final EndeffectorRollers endeffectorRollers;

  private WantedState wantedState = WantedState.STOPPED;
  private CurrentState currentState = CurrentState.STOPPED;

  public enum WantedState{
    RAMP_INTAKING,
    RAMP_HOLD_CORAL,
    ROLLERS_INTAKE,
    ENDEFFECTOR_HOLD_CORAL,
    ENDEFFECTOR_SCORE,
    STOPPED,
  }

  public enum CurrentState{
    RAMP_INTAKING,
    RAMP_HOLD_CORAL,
    ROLLERS_INTAKE,
    
    ENDEFFECTOR_HOLD_CORAL,
    ENDEFFECTOR_SCORE,
    STOPPED,
  }

  public Rollers(EndeffectorRollers endefRollers, RampRollers rampRollers) {
    this.rampRollers = rampRollers;
    this.endeffectorRollers = endefRollers;
  }

  public void updateInputs() {
    rampRollers.updateInputs();
    endeffectorRollers.updateInputs();
    handleStateTransitions();
    applyStates();
  }

  public void handleStateTransitions(){
    switch(wantedState){
      case STOPPED:
        currentState = CurrentState.STOPPED;
        break;
      case RAMP_INTAKING:
        if(rampRollers.isCoralDetected()){
          wantedState = WantedState.RAMP_HOLD_CORAL;
          currentState = CurrentState.RAMP_HOLD_CORAL;
        }
        else{
          currentState = CurrentState.RAMP_INTAKING;
        }
        break;
      case RAMP_HOLD_CORAL:
        if(!endeffectorRollers.isCoralDetected()){
          wantedState = WantedState.RAMP_INTAKING;
          currentState = CurrentState.RAMP_INTAKING;
        }
        else{
          currentState = CurrentState.RAMP_HOLD_CORAL;
        }
          break;
      case ROLLERS_INTAKE: {      
        if (!rampRollers.isCoralDetected() && !endeffectorRollers.isCoralDetected()) {
          // Nothing in the system yet
          currentState = CurrentState.ROLLERS_INTAKE;
        } else if (rampRollers.isCoralDetected() && !endeffectorRollers.isCoralDetected()) {
          // Coral is at the ramp only so we run BOTH to move it toward the end effector
          currentState = CurrentState.ROLLERS_INTAKE;
        } else if (rampRollers.isCoralDetected() && endeffectorRollers.isCoralDetected()) {
          // Coral is somewhere between the sensors 
          currentState = CurrentState.ROLLERS_INTAKE;
        } else {
          // Coral is only at the end effector now so we’re done transferring
          currentState = CurrentState.ENDEFFECTOR_HOLD_CORAL;
        }
        break;
      }          
      case ENDEFFECTOR_HOLD_CORAL:
        if(!endeffectorRollers.isCoralDetected()){
          wantedState = WantedState.ROLLERS_INTAKE;
          currentState = CurrentState.ROLLERS_INTAKE;
        }
        else{
          currentState = CurrentState.ENDEFFECTOR_HOLD_CORAL;
        }
        break;
      case ENDEFFECTOR_SCORE:
        if(!endeffectorRollers.isCoralDetected()){
          wantedState = WantedState.ROLLERS_INTAKE;
          currentState = CurrentState.ROLLERS_INTAKE;
        }
        else{
          currentState = CurrentState.ENDEFFECTOR_SCORE;
        }
        break;
      default:
        currentState = CurrentState.STOPPED;
        break;
    }
  }

  public void applyStates(){
    switch(currentState){
      case STOPPED:
        endeffectorRollers.setWantedState(EndeffectorRollers.WantedState.STOPPED);
        rampRollers.setWantedState(RampRollers.WantedState.STOPPED);
        break;
      case RAMP_INTAKING:
        endeffectorRollers.setWantedState(EndeffectorRollers.WantedState.STOPPED);
        rampRollers.setWantedState(RampRollers.WantedState.INTAKING);
        break;
      case RAMP_HOLD_CORAL:
        endeffectorRollers.setWantedState(EndeffectorRollers.WantedState.STOPPED);
        rampRollers.setWantedState(RampRollers.WantedState.HOLD_CORAL);
        break;
      case ROLLERS_INTAKE: // lowk my brain is fried rn and this makes the most sense to me
        endeffectorRollers.setWantedState(EndeffectorRollers.WantedState.SCORING_CORAL);
        rampRollers.setWantedState(RampRollers.WantedState.SCORING_CORAL);
        break;
      case ENDEFFECTOR_HOLD_CORAL:
        endeffectorRollers.setWantedState(EndeffectorRollers.WantedState.HOLD_CORAL);
        rampRollers.setWantedState(RampRollers.WantedState.STOPPED);
        break;
      case ENDEFFECTOR_SCORE:
        endeffectorRollers.setWantedState(EndeffectorRollers.WantedState.SCORING_CORAL);
        rampRollers.setWantedState(RampRollers.WantedState.STOPPED);
        break;
    }
  }

  public void setWantedState(WantedState wantedState){
    this.wantedState = wantedState;
  }
}
