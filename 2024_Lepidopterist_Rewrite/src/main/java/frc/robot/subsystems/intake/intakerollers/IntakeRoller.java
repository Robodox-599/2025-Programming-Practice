package frc.robot.subsystems.intake.intakerollers;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.intake.intakerollers.IntakeRollerConstants.IntakeRollerStates;

public class IntakeRoller {
  private final IntakeRollerIO io;
  private Timer beamBreakTimer = new Timer(); 
  private DigitalInput beamBreak;
  private WantedState wantedState = WantedState.STOP;
  private CurrentState currentState = CurrentState.STOP;
  private CurrentState previousState = CurrentState.STOP;

  public IntakeRoller(IntakeRollerIO io) {
    this.io = io;
    beamBreakTimer.start();
    beamBreak = new DigitalInput(IntakeRollerConstants.beamBreakPort);
  }

  public void periodic() {
      io.updateInputs();

      if(beamBreak.get())
      {
        beamBreakTimer.reset();
      }
  }

  public enum WantedState{
    INTAKING,
    SCORING,
    HOLDNOTE,
    STOP,
  }

  public enum CurrentState{
    INTAKING,
    SCORING,
    HOLDNOTE,
    STOP,
  }

  public void handleStateTransitions(){
    previousState = currentState;
    switch (wantedState) {
      case INTAKING:
        currentState = CurrentState.INTAKING;
        break;
      case SCORING:
        currentState = CurrentState.SCORING;
        break;
      case HOLDNOTE:
        currentState = CurrentState.HOLDNOTE;
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
          setVelocity(IntakeRollerStates.INTAKING);
          break;
        case SCORING:
          setVelocity(IntakeRollerStates.SCORING);
          break;
        case HOLDNOTE:
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