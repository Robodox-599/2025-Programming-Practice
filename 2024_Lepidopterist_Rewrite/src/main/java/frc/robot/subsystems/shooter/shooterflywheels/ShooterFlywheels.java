package frc.robot.subsystems.shooter.shooterflywheels;

import frc.robot.subsystems.intake.intakerollers.IntakeRollerConstants.IntakeRollerStates;
import frc.robot.subsystems.shooter.shooterflywheels.ShooterFlywheelsConstants;
import frc.robot.util.SubsystemChecker;

public class ShooterFlywheels {
  private final ShooterFlywheelsIO io;
  private WantedState wantedState = WantedState.STOP;
  private CurrentState currentState = CurrentState.STOP;
  private CurrentState previousState = CurrentState.STOP;
  private SubsystemChecker subsystemChecker;

  public ShooterFlywheels(ShooterFlywheelsIO io, SubsystemChecker subsystemChecker) {
    this.io = io;
    this.subsystemChecker = subsystemChecker;
  }

  public void periodic() {
      io.updateInputs();
      subsystemChecker.updateIntakeRollerDetector(isTopNoteDetected());
  }

  public enum WantedState{
    INTAKING,
    SCORING,
    HOLD_NOTE,
    STOP,
    NO_NOTE,
  }

  public enum CurrentState{
    SCORING,
    HOLD_NOTE,
    STOP,
    NO_NOTE,
  }

  public void handleStateTransitions(){
    previousState = currentState;
    switch (wantedState) {
      case SCORING:
        if(!isNoteDetected()){
          currentState = CurrentState.NO_NOTE;
        }
        else{
          currentState = CurrentState.SCORING;
        }
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
        case SCORING:
          setVelocity(ShooterFlywheelsConstants.ShooterFlywheelsStates.SCORING);
          break;
        case STOP:
          setVelocity(ShooterFlywheelsConstants.ShooterFlywheelsStates.STOP);
          break;
        default:
          setVelocity(ShooterFlywheelsConstants.ShooterFlywheelsStates.STOP);
          break;
      }
    }
  }

  public void setVelocity(ShooterFlywheelsConstants.ShooterFlywheelsStates state) {
    io.setVelocity(state);
  }

  public void stop() {
    io.stop();
  }

  public boolean isTopNoteDetected() {
    return io.topIsNoteDetected;
  }
}