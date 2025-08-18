package frc.robot.subsystems.indexer;

import dev.doglog.DogLog;

public class Indexer {
  private final IndexerIO io;

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  public void updateInputs() {
    io.updateInputs();
  }

  public void setVelocity(double velocity) {
    io.setVelocity(velocity);
  }

  public void stop() {
    io.stop();
  }

  public boolean noteDetected() {
    return io.noteDetected;
  }

  public boolean noteEnsured() {
    return io.noteEnsured;
  }
}