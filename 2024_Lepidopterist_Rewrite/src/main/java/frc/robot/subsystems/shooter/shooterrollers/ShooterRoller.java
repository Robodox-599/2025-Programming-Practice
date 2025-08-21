package frc.robot.subsystems.shooter.shooterrollers;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.shooter.shooterrollers.ShooterRollerConstants.ShooterRollerStates;

public class ShooterRoller {
  private final ShooterRollerIO io;
  private Timer beamBreakTimer = new Timer(); 
  private DigitalInput beamBreak;

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