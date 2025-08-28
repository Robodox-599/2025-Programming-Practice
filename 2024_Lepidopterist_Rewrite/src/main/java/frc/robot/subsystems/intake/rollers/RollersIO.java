package frc.robot.subsystems.intake.rollers;

public class RollersIO {
  
    protected double appliedVoltage = 0.0;
    protected double velocity = 0.0;
    protected double statorCurrentAmps = 0.0;
    protected double supplyCurrentAmps = 0.0;
    protected double tempCelsius = 0.0;
  
    public void updateInputs() {}
    public void setVelocity(double velocity) {}
    public void stop() {}
    public double getSpeed() {
      return velocity;
    }
  }
  