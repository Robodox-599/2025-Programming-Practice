// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.rollers;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.endeffectorrollers.EndeffectorRollers;
import frc.robot.subsystems.ramprollers.RampRollers;

public class Rollers extends SubsystemBase {
  
  private final RampRollers rampRollers;
  private final EndeffectorRollers endeffectorRollers;

  private WantedState wantedState = WantedState.STOPPED;
  private CurrentState currentState = CurrentState.STOPPED;

  public enum WantedState{
    ROLLERS_INTAKE_CORAL,
    ENDEFFECTOR_INTAKE_ALGAE,
    ENDEFFECTOR_HOLD_CORAL,
    ENDEFFECTOR_HOLD_ALGAE,
    ENDEFFECTOR_SCORE_ALGAE,
    ENDEFFECTOR_SCORE_CORAL,
    STOPPED,
  }

  public enum CurrentState{
    ROLLERS_INTAKE_CORAL,
    ENDEFFECTOR_INTAKE_ALGAE,
    ENDEFFECTOR_HOLD_CORAL,
    ENDEFFECTOR_HOLD_ALGAE,
    ENDEFFECTOR_SCORE_ALGAE,
    ENDEFFECTOR_SCORE_CORAL,
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

    DogLog.log("Rollers/wantedState", wantedState);
    DogLog.log("Rollers/currentState", currentState);
  }

  public void handleStateTransitions(){
    switch(wantedState){
      case ROLLERS_INTAKE_CORAL: {      
        if (!rampRollers.isCoralDetected() && !endeffectorRollers.isCoralDetected()) {
          // Nothing in the system yet
          currentState = CurrentState.ROLLERS_INTAKE_CORAL;
        } else if (rampRollers.isCoralDetected() && !endeffectorRollers.isCoralDetected()) {
          // Coral is at the ramp only so we run BOTH to move it toward the end effector
          currentState = CurrentState.ROLLERS_INTAKE_CORAL;
        } else if (rampRollers.isCoralDetected() && endeffectorRollers.isCoralDetected()) {
          // Coral is somewhere between the sensors 
          currentState = CurrentState.ROLLERS_INTAKE_CORAL;
        } else {
          // Coral is only at the end effector now so we’re done transferring
          currentState = CurrentState.ENDEFFECTOR_HOLD_CORAL;
        }
      }          
      break;
      case ENDEFFECTOR_INTAKE_ALGAE:
        if(endeffectorRollers.isAlgaeDetected()){
          wantedState = WantedState.ENDEFFECTOR_HOLD_ALGAE;
          currentState = CurrentState.ENDEFFECTOR_HOLD_ALGAE;
        } else{
          currentState = CurrentState.ENDEFFECTOR_INTAKE_ALGAE;
        }
        break;
      case ENDEFFECTOR_HOLD_CORAL:
        if(!endeffectorRollers.isCoralDetected()){
          wantedState = WantedState.ROLLERS_INTAKE_CORAL;
          currentState = CurrentState.ROLLERS_INTAKE_CORAL;
        }
        else{
          currentState = CurrentState.ENDEFFECTOR_HOLD_CORAL;
        }
        break;
      case ENDEFFECTOR_HOLD_ALGAE:
        if(!endeffectorRollers.isAlgaeDetected()){
          wantedState = WantedState.ENDEFFECTOR_INTAKE_ALGAE;
          currentState = CurrentState.ENDEFFECTOR_INTAKE_ALGAE;
        } else{
          currentState = CurrentState.ENDEFFECTOR_HOLD_ALGAE;
        }
        break;
      case ENDEFFECTOR_SCORE_CORAL:
        if(!endeffectorRollers.isCoralDetected()){
          wantedState = WantedState.ROLLERS_INTAKE_CORAL;
          currentState = CurrentState.ROLLERS_INTAKE_CORAL;
        }
        else{
          currentState = CurrentState.ENDEFFECTOR_SCORE_CORAL;
        }
        break;
      case ENDEFFECTOR_SCORE_ALGAE:
        if(!endeffectorRollers.isAlgaeDetected()){
          wantedState = WantedState.STOPPED;
          currentState = CurrentState.STOPPED;
        } else{
          currentState = CurrentState.ENDEFFECTOR_SCORE_ALGAE;
        }
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
      case ROLLERS_INTAKE_CORAL: // lowk my brain is fried rn and this makes the most sense to me
        endeffectorRollers.setWantedState(EndeffectorRollers.WantedState.INTAKING_CORAL);
        rampRollers.setWantedState(RampRollers.WantedState.INTAKING);
        break;
      case ENDEFFECTOR_INTAKE_ALGAE:
        endeffectorRollers.setWantedState(EndeffectorRollers.WantedState.INTAKING_ALGAE);
        break;
      case ENDEFFECTOR_HOLD_CORAL:
        endeffectorRollers.setWantedState(EndeffectorRollers.WantedState.HOLD_CORAL);
        break;
      case ENDEFFECTOR_HOLD_ALGAE:
        endeffectorRollers.setWantedState(EndeffectorRollers.WantedState.HOLD_ALGAE);
        break;
      case ENDEFFECTOR_SCORE_CORAL:
        endeffectorRollers.setWantedState(EndeffectorRollers.WantedState.SCORING_CORAL);
        break;
      case ENDEFFECTOR_SCORE_ALGAE:
        endeffectorRollers.setWantedState(EndeffectorRollers.WantedState.SCORING_ALGAE);
        break;        
      case STOPPED:
        endeffectorRollers.setWantedState(EndeffectorRollers.WantedState.STOPPED);
        break;
      default:
        endeffectorRollers.setWantedState(EndeffectorRollers.WantedState.STOPPED);
        break;
    }
  }

  public void setWantedState(WantedState wantedState){
    this.wantedState = wantedState;
  }
}
