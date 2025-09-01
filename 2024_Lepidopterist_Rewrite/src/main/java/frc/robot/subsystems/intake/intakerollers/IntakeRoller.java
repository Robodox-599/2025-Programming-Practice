package frc.robot.subsystems.intake.intakerollers;

import frc.robot.subsystems.intake.intakerollers.IntakeRollerConstants.IntakeRollerStates;
import frc.robot.util.SubsystemChecker;

public class IntakeRoller {
  private final IntakeRollerIO io;
  private WantedState wantedState = WantedState.STOP;
  private CurrentState currentState = CurrentState.STOP;
  private CurrentState previousState = CurrentState.STOP;
  private SubsystemChecker subsystemChecker;

  public IntakeRoller(IntakeRollerIO io, SubsystemChecker subsystemChecker) {
    this.io = io;
    this.subsystemChecker = subsystemChecker;
  }

  public void periodic() {
      io.updateInputs();
      subsystemChecker.updateIntakeRollerDetector(isNoteDetected());
  }

  public enum WantedState{
    INTAKING,
    SCORING,
    HOLD_NOTE,
    STOP,
    NO_NOTE,
  }

  public enum CurrentState{
    INTAKING,
    SCORING,
    HOLD_NOTE,
    STOP,
    NO_NOTE,
  }

  public void handleStateTransitions(){
    previousState = currentState;
    switch (wantedState) {
      case INTAKING:
        if(isNoteDetected()){
          currentState = CurrentState.HOLD_NOTE;
        }
        else{
          currentState = CurrentState.INTAKING;
        }
        break;
      case SCORING:
        if(!isNoteDetected()){
          currentState = CurrentState.NO_NOTE;
        }
        else{
          currentState = CurrentState.SCORING;
        }
        break;
      case HOLD_NOTE:
        currentState = CurrentState.HOLD_NOTE;
        break;
      case STOP:
        currentState = CurrentState.STOP;
        break;
      case NO_NOTE:
        if(!isNoteDetected()){
          currentState = CurrentState.NO_NOTE;
        }
        else{
          currentState = CurrentState.HOLD_NOTE;
        }
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
          setVelocity(IntakeRollerStates.INTAKING);
          break;
        case SCORING:
          setVelocity(IntakeRollerStates.SCORING);
          break;
        case HOLD_NOTE:
          setVelocity(IntakeRollerStates.HOLDNOTE);
          break;
        case STOP:
          setVelocity(IntakeRollerStates.STOP);
          break;
        default:
          setVelocity(IntakeRollerStates.STOP);
          break;
      }
    }
  }

  public void setVelocity(IntakeRollerStates state) {
    io.setVelocity(state);
  }

  public void stop() {
    io.stop();
  }

  public boolean isNoteDetected() {
    return io.isNoteDetected;
  }
}