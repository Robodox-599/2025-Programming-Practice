// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector.endeffectorrollers;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndeffectorRollers extends SubsystemBase {
  private final EndeffectorRollersIO io;
  private WantedState wantedState = WantedState.STOPPED;
  private CurrentState currentState = CurrentState.STOPPED;

  public enum WantedState{
    INTAKING_CORAL,
    HOLD_CORAL,
    SCORING_CORAL,
    STOPPED,
  }

  public enum CurrentState{
    INTAKING_CORAL,
    HOLD_CORAL,
    SCORING_CORAL,
    STOPPED,
  }

  public EndeffectorRollers(EndeffectorRollersIO io) {
    this.io = io;
  }

  public void updateInputs() {
    handleStateTransitions();
    applyStates();

    DogLog.log("EndeffectorRollers/wantedState", wantedState);
    DogLog.log("EndeffectorRollers/currentState", currentState);

    io.updateInputs();
  }

  public void handleStateTransitions(){
    switch(wantedState){
      case STOPPED:
        currentState = CurrentState.STOPPED;
        break;
      case INTAKING_CORAL:
        if(isCoralDetected()){
          wantedState = WantedState.HOLD_CORAL;
          currentState = CurrentState.HOLD_CORAL;
        }
        else{
          currentState = CurrentState.INTAKING_CORAL;
        }
        break;
      case HOLD_CORAL:
        if(!isCoralDetected()){
          wantedState = WantedState.INTAKING_CORAL;
          currentState = CurrentState.INTAKING_CORAL;
        }
        else{
          currentState = CurrentState.HOLD_CORAL;
        }
      case SCORING_CORAL:
        if(!isCoralDetected()){
          wantedState = WantedState.INTAKING_CORAL;
          currentState = CurrentState.INTAKING_CORAL;
        }
        else{
          currentState = CurrentState.SCORING_CORAL;
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
        stop();
        break;
      case INTAKING_CORAL:
        setVelocity(-0.3);
        break;
      case SCORING_CORAL:
        setVelocity(-0.5);
        break;
      case HOLD_CORAL:
        setPosition(io.heldCurrentPosition);
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
