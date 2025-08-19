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

      // if the beambreak detects the note, reset the checking timer
      if(beamBreak.get())
      {
        beamBreakTimer.reset();
      }
  }

  // changes the velocity depending on the state we're switching to
  public void setVelocity(IntakeRollerStates state) {
    io.setVelocity(state);
  }

  // stops the motor and logging
  public void stop() {
    io.stop();
  }

  // checks whether the note is still being detected by the beambreak
  public boolean isNoteDetected() {
    return io.isNoteDetected;
  }
}