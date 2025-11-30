// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector.endeffectorrollers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndeffectorRollers extends SubsystemBase {
  private final EndeffectorRollersIO io;

  private WantedState wantedState = WantedState.STOPPED;
  private CurrentState currentState = CurrentState.STOPPED;

  public enum WantedState{
    INTAKING_CORAL_STATION,
    INTAKING_ALGAE,
    SCORING_CORAL_TROUGH,
    SCORING_CORAL_L2,
    SCORING_CORAL_L3,
    SCORING_CORAL_L4,
    SCORING_ALGAE,
    HOLD_CORAL,
    HOLD_ALGAE,
    STOPPED,
  }

  public enum CurrentState{
    INTAKING_CORAL_STATION,
    INTAKING_ALGAE,
    SCORING_CORAL_TROUGH,
    SCORING_CORAL_L2,
    SCORING_CORAL_L3,
    SCORING_CORAL_L4,
    SCORING_ALGAE,
    HOLD_CORAL,
    HOLD_ALGAE,
    STOPPED,
  }

  /** Creates a new RampRollers. */
  public EndeffectorRollers(EndeffectorRollersIO io) {
    this.io = io;
  }

  public void updateInputs() {
    handleStateTransitions();
    applyStates();

    io.updateInputs();
  }

  public void handleStateTransitions(){
    switch(wantedState){
      case INTAKING_CORAL_STATION:
        if(isCoralDetected()){
          wantedState = WantedState.HOLD_CORAL;
          currentState = CurrentState.HOLD_CORAL;
        }
        else{
          currentState = CurrentState.INTAKING_CORAL_STATION;
        }
        break;
      case INTAKING_ALGAE:
        if(isCoralDetected()){
          wantedState = WantedState.HOLD_CORAL;
          currentState = CurrentState.HOLD_CORAL;
        }
        else{
          currentState = CurrentState.INTAKING_ALGAE;
        }
        break;
      case SCORING_CORAL_TROUGH:
        currentState = CurrentState.SCORING_CORAL_TROUGH;
        break;
      case SCORING_CORAL_L2:
        currentState = CurrentState.SCORING_CORAL_L2;
        break;
      case SCORING_CORAL_L3:
        currentState = CurrentState.SCORING_CORAL_L3;
        break;
      case SCORING_CORAL_L4:
        currentState = CurrentState.SCORING_CORAL_L4;
        break;
      case SCORING_ALGAE:
        currentState = CurrentState.SCORING_ALGAE;
        break;
      case HOLD_CORAL:
        if(!isCoralDetected()){
          wantedState = WantedState.STOPPED;
          currentState = CurrentState.STOPPED;
        }
        else{
          currentState = CurrentState.HOLD_CORAL;
        }
        break;
      case HOLD_ALGAE:
        if(isCoralDetected()){
          wantedState = WantedState.HOLD_CORAL;
          currentState = CurrentState.HOLD_CORAL;
        }
        else{
          currentState = CurrentState.HOLD_ALGAE;
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
      case INTAKING_CORAL_STATION:
        stop();
        break;
      case INTAKING_ALGAE:
        setVelocity(-0.7);
        break;
      case SCORING_CORAL_TROUGH:
        setVelocity(-0.3);
        break;
      case SCORING_CORAL_L2:
        setVelocity(-0.5);
        break;
      case SCORING_CORAL_L3:
        setVelocity(-0.5);
        break;
      case SCORING_CORAL_L4:
        setVelocity(-0.8);
        break;
      case SCORING_ALGAE:
        setVelocity(-0.9);
        break;
      case HOLD_CORAL:
        setPosition(io.heldCurrentPosition);
        break;
      case HOLD_ALGAE:
        setPosition(io.heldCurrentPosition);
        break;
      case STOPPED:
        setVelocity(0);
        break;
      default:
        setVelocity(0);
        break;
    }
  }

  public void setVelocity(double velocity){
    io.setVelocity(velocity);
  }

  public void stop(){
    io.stop();
  }

  public void setWantedState(WantedState wantedState){
    this.wantedState = wantedState;
  }

  public boolean isCoralDetected(){
    return io.isCoralDetected;
  }

  public void setPosition(double position){
    io.setPosition(position);
  }
}
