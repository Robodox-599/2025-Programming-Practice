// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffectorwrist;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndeffectorWrist extends SubsystemBase {
   private final EndeffectorWristIO io;
  private WantedState wantedState = WantedState.STOPPED;
  private CurrentState currentState = CurrentState.STOPPED;

  public EndeffectorWrist(EndeffectorWristIO io) {
    this.io = io;
  }

  public enum WantedState {
    STOPPED,
    PREPARED,
    INTAKING_ALGAE_REEF,
    INTAKING_ALGAE_GROUND,
    SCORING_CORAL,
    SCORING_ALGAE_BARGE,
    INTAKING_CORAL,
    SCORING_ALGAE_PROCESSOR,
  }

  public enum CurrentState {
    STOPPED,
    PREPARED,
    INTAKING_ALGAE_REEF,
    INTAKING_ALGAE_GROUND,
    SCORING_CORAL,
    SCORING_ALGAE_BARGE,
    INTAKING_CORAL,
    SCORING_ALGAE_PROCESSOR,
  }

  public void updateInputs() {
    io.updateInputs();

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
        case PREPARED:
            currentState = CurrentState.PREPARED;
            break;
        case STOPPED:
            currentState = CurrentState.STOPPED;
            break;
        default:
            currentState = CurrentState.STOPPED;
            break;
    }
  }

  private void applyStates() { // I'll figure out the exact angles later
    switch (currentState) {
        case INTAKING_ALGAE_REEF:
            setPosition(-0.1);
            break;
        case INTAKING_ALGAE_GROUND:
            setPosition(-0.1);
            break;
        case SCORING_CORAL:
            setPosition(-0.14);
            break;
        case INTAKING_CORAL:
            setPosition(-0.3);
            break;
        case SCORING_ALGAE_BARGE:
            setPosition(-0.21);
            break;
        case SCORING_ALGAE_PROCESSOR:
            setPosition(-0.21);
            break;
        case PREPARED:
            setPosition(-0.21);
            break;
        case STOPPED:
            stop();
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
