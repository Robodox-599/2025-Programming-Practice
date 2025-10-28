// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.shooterwrist;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.shooterwrist.ShooterWristConstants.ShooterWristStates;

public class ShooterWrist extends SubsystemBase {
  private final ShooterWristIO io;
  private WantedState wantedState = WantedState.STOP;
  private CurrentState currentState = CurrentState.STOP;
  private CurrentState previousState = CurrentState.STOP;

  public ShooterWrist(ShooterWristIO io) {
    this.io = io;
  }

  public void periodic(){
    io.updateInputs();
  }

  public enum WantedState{
    INTAKING,
    STOW,
    STOP,
  }

  public enum CurrentState{
    INTAKING,
    STOW,
    STOP,
  }

  public void handleStateTransitions(){
    previousState = currentState;
    switch (wantedState) {
      case INTAKING:
        currentState = CurrentState.INTAKING;
        break;
      case STOW:
        currentState = CurrentState.STOW;
        break;
      case STOP:
        currentState = CurrentState.STOP;
        break;
      default:
        currentState = CurrentState.STOP;
        break;
    }
  }

  public void applyStates(){
    if(previousState != currentState){
      switch (currentState) {
        case INTAKING:
          setAngle(ShooterWristStates.INTAKING);
          break;
        case STOW:
          setAngle(ShooterWristStates.STOW);
          break;
        case STOP:
          setAngle(ShooterWristStates.STOP);
          break;
        default:
          setAngle(ShooterWristStates.STOP);
          break;
      }
    }
  }

  public void setAngle(ShooterWristConstants.ShooterWristStates state) {
    io.setAngle(state);
  }

  public boolean isAtSetpoint() {
    return io.atSetpoint;
  }
  
  public void stop() {
    io.stop();
  }

  public boolean isAtPrepScoreSetpoint(){
    return io.isAtPrepScoreSetpoint;
  }
}
