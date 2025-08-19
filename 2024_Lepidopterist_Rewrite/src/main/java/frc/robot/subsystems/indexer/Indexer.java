package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.indexer.IndexerConstants.IndexerStates;

public class Indexer {
  private final IndexerIO io;
  private Timer beamBreakTimer = new Timer(); 
  private DigitalInput beamBreak;


  public Indexer(IndexerIO io) {
    this.io = io;
    beamBreakTimer.start();
    beamBreak = new DigitalInput(IndexerConstants.beamBreakPort);
  }

  public void periodic() {
      io.updateInputs();

      if(beamBreak.get())
      {
        beamBreakTimer.reset();
      }
  }

  public void setVelocity(IndexerStates state) {
    io.setVelocity(state);
  }

  public void stop() {
    io.stop();
  }

  public boolean isNoteDetected() {
    return io.isNoteDetected;
  }
}