// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.rollers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.rollers.RampRollers.RampRollers;
import frc.robot.Subsystems.rollers.RampRollers.RampRollers.WantedState;
//import frc.robot.Subsystems.rollers.endeffectorrollers.WantedState;
import frc.robot.Subsystems.rollers.endeffectorrollers.EndeffectorRollers;

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

  @Override
  public void periodic() {
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
      case ROLLERS_INTAKE:
        if(!rampRollers.isCoralDetected()){
          wantedState = WantedState.RAMP_HOLD_CORAL;
          currentState = CurrentState.RAMP_HOLD_CORAL;
        }
        else{
          while (rampRollers.isCoralDetected() && endeffectorRollers.isCoralDetected()) {
            currentState = CurrentState.ROLLERS_INTAKE;

            if(!rampRollers.isCoralDetected()){
              break;
            }
          }
        }
        break;
      case ENDEFFECTOR_HOLD_CORAL:
        if(endeffectorRollers.isCoralDetected()){
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
        endeffectorRollers.setWantedState(frc.robot.Subsystems.rollers.endeffectorrollers.EndeffectorRollers.WantedState.STOPPED);
        rampRollers.setWantedState(frc.robot.Subsystems.rollers.RampRollers.RampRollers.WantedState.STOPPED);
        break;
      case RAMP_INTAKING:
        endeffectorRollers.setWantedState(frc.robot.Subsystems.rollers.endeffectorrollers.EndeffectorRollers.WantedState.STOPPED);
        rampRollers.setWantedState(frc.robot.Subsystems.rollers.RampRollers.RampRollers.WantedState.INTAKING);
        break;
      case RAMP_HOLD_CORAL:
        endeffectorRollers.setWantedState(frc.robot.Subsystems.rollers.endeffectorrollers.EndeffectorRollers.WantedState.STOPPED);
        rampRollers.setWantedState(frc.robot.Subsystems.rollers.RampRollers.RampRollers.WantedState.HOLD_CORAL);
        break;
      case ROLLERS_INTAKE: // lowk my brain is fried rn and this makes the most sense to me
        endeffectorRollers.setWantedState(frc.robot.Subsystems.rollers.endeffectorrollers.EndeffectorRollers.WantedState.SCORING_CORAL);
        rampRollers.setWantedState(frc.robot.Subsystems.rollers.RampRollers.RampRollers.WantedState.SCORING_CORAL);
        break;
      case ENDEFFECTOR_HOLD_CORAL:
        endeffectorRollers.setWantedState(frc.robot.Subsystems.rollers.endeffectorrollers.EndeffectorRollers.WantedState.HOLD_CORAL);
        rampRollers.setWantedState(frc.robot.Subsystems.rollers.RampRollers.RampRollers.WantedState.STOPPED);
        break;
      case ENDEFFECTOR_SCORE:
        endeffectorRollers.setWantedState(frc.robot.Subsystems.rollers.endeffectorrollers.EndeffectorRollers.WantedState.SCORING_CORAL);
        rampRollers.setWantedState(frc.robot.Subsystems.rollers.RampRollers.RampRollers.WantedState.STOPPED);
        break;
    }
  }
}
