package frc.robot.subsystems.shooter.flywheels;

public class FlywheelsIO {
  
  protected double topAppliedVoltage = 0.0;
  protected double topVelocity = 0.0;
  protected double topStatorCurrentAmps = 0.0;
  protected double topSupplyCurrentAmps = 0.0;
  protected double topTempCelsius = 0.0;

  protected double bottomAppliedVoltage = 0.0;
  protected double bottomVelocity = 0.0;
  protected double bottomStatorCurrentAmps = 0.0;
  protected double bottomSupplyCurrentAmps = 0.0;
  protected double bottomTempCelsius = 0.0;

  protected double wantedVelocity = 0.0;

  protected boolean areFlywheelsAtSpeed = false;

  public void updateInputs() {}
  public void setVelocity(double velocity) {}
  public void stop() {}
  public double getTopVelocity() {
    return topVelocity;
  }
}
