// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.RampRollers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RampRollers {
  private final RampRollersIO io;
  private WantedState wantedState = WantedState.STOPPED;
  private CurrentState currentState = CurrentState.STOPPED;

  public enum WantedState{
    INTAKING,
    HOLD_CORAL,
    SCORING,
    STOPPED,
  }

  public enum CurrentState{
    INTAKING,
    HOLD_CORAL,
    SCORING,
    STOPPED,
  }

  /** Creates a new RampRollers. */
  public RampRollers(RampRollersIO io) {
    this.io = io;
  }

  public void updateInputs() {
    handleStateTransitions();
    applyStates();
  }

  public void handleStateTransitions(){
    switch(wantedState){
      case STOPPED:
        currentState = CurrentState.STOPPED;
        break;
      case INTAKING:
        if(isCoralDetected()){
          wantedState = WantedState.HOLD_CORAL;
          currentState = CurrentState.HOLD_CORAL;
        }
        else{
          currentState = CurrentState.INTAKING;
        }
        break;
      case HOLD_CORAL:
        if(!isCoralDetected()){
          wantedState = WantedState.INTAKING;
          currentState = CurrentState.INTAKING;
        }
        else{
          currentState = CurrentState.HOLD_CORAL;
        }
      case SCORING:
        if(!isCoralDetected()){
          wantedState = WantedState.INTAKING;
          currentState = CurrentState.INTAKING;
        }
        else{
          currentState = CurrentState.SCORING;
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
      case INTAKING:
        setVelocity(0.5);
        break;
      case SCORING:
        setVelocity(2);
        break;
      case HOLD_CORAL:
        setPosition(currentPosition());
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

  public double currentPosition(){
    return io.position;
  }

  public void setPosition(double position){
    io.setPosition(position);
  }

}
