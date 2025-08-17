package frc.robot.subsystems.indexer;

public class IndexerIO {
  
  protected double appliedVoltage = 0.0;
  protected double velocity = 0.0;
  protected double statorCurrentAmps = 0.0;
  protected double supplyCurrentAmps = 0.0;
  protected double tempCelsius = 0.0;

  protected double wantedVelocity = 0.0; 
  protected boolean noteDetected = false;
  protected boolean noteEnsured = false;

  public void updateInputs() {}

  public void setVelocity(double velocity) {}

  public double getVelocity() {
    return velocity;
  }
  public void stop() {}

  public void noteDetected() {}
}
