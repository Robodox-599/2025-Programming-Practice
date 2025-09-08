package frc.robot.subsystems.shooter.wrist;

public class ShooterWristIO {
    protected double appliedVoltage = 0;
    protected double velocity = 0;
    protected double currentAmps = 0;
    protected double tempCelsius = 0;
    
    protected double currentPosition = 0;
    protected double targetPosition = 0;

  public void updateInputs() {}
  public void stop() {}
  public void setBrake(boolean brake) {}
  public void goToAngle(double angle) {}
  public void holdAngle(double angle) {}
  public void setVoltage(double voltage) {}
}