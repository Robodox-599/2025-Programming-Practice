// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffectorwrist;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndeffectorWrist extends SubsystemBase {
   private final EndeffectorWristIO io;
  private WantedState wantedState = WantedState.STOW;
  private CurrentState currentState = CurrentState.STOW;

  public EndeffectorWrist(EndeffectorWristIO io) {
    this.io = io;
  }

  public enum WantedState {
    STOW,
    INTAKING_ALGAE_REEF,
    INTAKING_ALGAE_GROUND,
    SCORING_CORAL,
    SCORING_ALGAE_BARGE,
    SCORING_ALGAE_PROCESSOR,
  }

  public enum CurrentState {
    STOW,
    INTAKING_ALGAE_REEF,
    INTAKING_ALGAE_GROUND,
    SCORING_CORAL,
    SCORING_ALGAE_BARGE,
    SCORING_ALGAE_PROCESSOR,
  }

  public void updateInputs() {
    handleStateTransitions();
    applyStates();

    DogLog.log("Endeffector/Wrist/CurrentState", currentState);
    DogLog.log("Endeffector/Wrist/WantedState", wantedState);
  }

  private void handleStateTransitions() {
    switch (wantedState) {
        case INTAKING_ALGAE_REEF:
            currentState = CurrentState.INTAKING_ALGAE_REEF;
            break;
        case INTAKING_ALGAE_GROUND:
            currentState = CurrentState.INTAKING_ALGAE_GROUND;
            break;
        case SCORING_CORAL:
            currentState = CurrentState.SCORING_CORAL;
            break;
        case SCORING_ALGAE_BARGE:
            currentState = CurrentState.SCORING_ALGAE_BARGE;
            break;
        case SCORING_ALGAE_PROCESSOR:
            currentState = CurrentState.SCORING_ALGAE_PROCESSOR;
            break;
        case STOW:
            currentState = CurrentState.STOW;
            break;
        default:
            currentState = CurrentState.STOW;
            break;
    }
  }

  private void applyStates() { // I'll figure out the exact angles later
    switch (currentState) {
        case INTAKING_ALGAE_REEF:
            setPosition(0);
            break;
        case INTAKING_ALGAE_GROUND:
            setPosition(0);
            break;
        case SCORING_CORAL:
            setPosition(0);
            break;
        case SCORING_ALGAE_BARGE:
            setPosition(0);
            break;
        case SCORING_ALGAE_PROCESSOR:
            setPosition(0);
            break;
        case STOW:
            setPosition(0);
            break;
        default:
            stop();
            break;
    }
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  public boolean isWristInPosition(){
    return io.isWristInPosition;
  }

  public void setPosition(double angle){
    io.setPosition(angle);
  }

  public void stop() {
    io.stop();
  }
}
