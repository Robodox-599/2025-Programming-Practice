// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.rampRollers;

import dev.doglog.DogLog;

public class RampRollers {
  private final RampRollersIO io;
  private WantedState wantedState = WantedState.STOPPED;
  private CurrentState currentState = CurrentState.STOPPED;
  private boolean previousIsCoralDetected = false;
  private boolean currentIsCoralDetected = false;
  public double wantedCoralPosition;

  public enum WantedState{
    STOPPED,
    INTAKING,
    HOLD_CORAL,
    SCORING,
  }

  public enum CurrentState{
    STOPPED,
    INTAKING,
    HOLD_CORAL,
    SCORING,
  }

  /** Makes it so that you can acess RampRolelrs io */
  public RampRollers(RampRollersIO io){
    //this calls the one from the class (global variable)
    //clicking on it shows which one ur refering to
    this.io = io;

  }

  public void updateInputs() { //runs every 0.02 sec
    //updateInputs first
    io.updateInputs();
    handleStateTransitions();
    applyStates();
    previousIsCoralDetected = currentIsCoralDetected;
    currentIsCoralDetected = isCoralDetected();
    DogLog.log("RampRollers/wantedState", wantedState);
    DogLog.log("RampRollers/currentState", currentState);
  }

  private void handleStateTransitions() {
    switch (wantedState){
      case STOPPED:
        currentState = CurrentState.STOPPED;
      break;
      case INTAKING:
        currentState = CurrentState.INTAKING;
      break;
      case SCORING:
        currentState = CurrentState.SCORING;
      break;
      case HOLD_CORAL:
        currentState = CurrentState.HOLD_CORAL;
    }
  }


  private void applyStates() {
    switch (currentState) {
      case STOPPED:
        stop();
        break;
      case INTAKING:
        setVelocity(-0.15);
        break;
      case SCORING:
        setVelocity(-0.15);
        break;
      case HOLD_CORAL:
        setPosition(io.wantedCoralPosition);
      default:
        stop();
        break;
    }
  }

  public void stop(){
    io.stop();
  }

  public void setVelocity(double velocity) {
    io.setVelocity(velocity);
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  public boolean isCoralDetected() {
    return io.isCoralDetected;
  }

  public void setPosition(double position){
    io.setPosition(position);
  }
}
