package frc.robot.subsystems.indexer;

public abstract class IndexerIO {
  protected double tempCelsius = 0.0;
  protected double statorCurrentAmps = 0.0;
  protected double supplyCurrentAmps = 0.0;

  protected double appliedVolts = 0.0;
  protected double velocity = 0.0;
  protected double desiredVelocity = 0.0;
  protected boolean atSetSpeed = false;
  protected boolean isNoteDetected = false;
  protected IndexerConstants.IndexerStates state = IndexerConstants.IndexerStates.STOP;

  public void updateInputs() {}

  public void stop() {}

  public void setState(IndexerConstants.IndexerStates state){}

  public void setVelocity(IndexerConstants.IndexerStates state) {}

  public double getVelocity() {
    return velocity;
  }
}