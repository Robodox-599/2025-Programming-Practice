package frc.robot.subsystems.shooter.shooterflywheels;

import frc.robot.subsystems.shooter.shooterflywheels.ShooterFlywheelsConstants;

public abstract class ShooterFlywheelsIO {
  protected double tempCelsius = 0.0;
  protected double statorCurrentAmps = 0.0;
  protected double supplyCurrentAmps = 0.0;

  protected double appliedVolts = 0.0;
  protected double velocity = 0.0;
  protected double desiredVelocity = 0.0;
  protected boolean atSetSpeed = false;
  protected boolean isNoteDetected = false;
  protected ShooterFlywheelsConstants.ShooterFlywheelsStates state = ShooterFlywheelsConstants.ShooterFlywheelsStates.STOP;

  public void updateInputs() {}

  public void stop() {}

  public void setVelocity(ShooterFlywheelsConstants.ShooterFlywheelsStates state) {}

  public double getVelocity() {
    return velocity;
  }
}