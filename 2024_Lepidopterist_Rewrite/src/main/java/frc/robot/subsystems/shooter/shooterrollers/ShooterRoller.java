package frc.robot.subsystems.shooter.shooterrollers;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.shooter.shooterrollers.ShooterRollerConstants.ShooterRollerStates;

public class ShooterRoller {
  private final ShooterRollerIO io;
  private Timer beamBreakTimer = new Timer(); 
  private DigitalInput beamBreak;
  private WantedState wantedState = WantedState.STOP;
  private CurrentState currentState = CurrentState.STOP;
  private CurrentState previousState = CurrentState.STOP;

  public ShooterRoller(ShooterRollerIO io) {
    this.io = io;
    beamBreakTimer.start();
    beamBreak = new DigitalInput(ShooterRollerConstants.beamBreakPort);
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
          setVelocity(ShooterRollerStates.INTAKING);
          break;
        case SCORING:
          setVelocity(ShooterRollerStates.SCORING);
          break;
        case HOLDNOTE:
          setVelocity(ShooterRollerStates.HOLDNOTE);
          break;
        case STOP:
          setVelocity(ShooterRollerStates.STOP);
          break;
        default:
          setVelocity(ShooterRollerStates.STOP);
          break;
      }
    }
  }

  public void setVelocity(ShooterRollerStates state) {
    io.setVelocity(state);
  }

  public void stop() {
    io.stop();
  }

  public boolean isNoteDetected() {
    return io.isNoteDetected;
  }
}