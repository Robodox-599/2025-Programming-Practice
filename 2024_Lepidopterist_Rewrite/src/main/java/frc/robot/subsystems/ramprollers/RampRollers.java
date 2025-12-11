// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ramprollers;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RampRollers extends SubsystemBase {
  private final RampRollersIO io;
  private WantedState wantedState = WantedState.STOPPED;
  private CurrentState currentState = CurrentState.STOPPED;

  public enum WantedState{
    INTAKING,
    HOLD_CORAL,
    TRANSFER_CORAL,
    STOPPED,
  }

  public enum CurrentState{
    INTAKING,
    HOLD_CORAL,
    TRANSFER_CORAL,
    STOPPED,
  }

  /** Creates a new RampRollers. */
  public RampRollers(RampRollersIO io) {
    this.io = io;
  }

  public void updateInputs() {
    handleStateTransitions();
    applyStates();

    DogLog.log("RampRollers/wantedStated", wantedState);
    DogLog.log("RampRollers/currentState", currentState);

    io.updateInputs();
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
        }else{
          currentState = CurrentState.INTAKING;
        }
        break;
      case HOLD_CORAL:
        if(!isCoralDetected()){
          wantedState = WantedState.INTAKING;
          currentState = CurrentState.INTAKING;
        }else{
          currentState = CurrentState.HOLD_CORAL;
        }
        break;
      case TRANSFER_CORAL:
        if(!isCoralDetected()){
          wantedState = WantedState.INTAKING;
          currentState = CurrentState.INTAKING;
        } else{
          currentState = CurrentState.TRANSFER_CORAL;
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
        setVelocity(0.3);
        break;
      case TRANSFER_CORAL:
        setVelocity(0.3);
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
