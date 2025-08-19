package frc.robot.subsystems.intake.intakerollers;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.intake.intakerollers.IntakeRollerConstants.IntakeRollerStates;

public class IntakeRoller {
  private final IntakeRollerIO io;
  private Timer beamBreakTimer = new Timer(); 
  private DigitalInput beamBreak;


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