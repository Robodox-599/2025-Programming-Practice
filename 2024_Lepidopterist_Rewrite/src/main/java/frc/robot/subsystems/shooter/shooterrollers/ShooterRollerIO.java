package frc.robot.subsystems.shooter.shooterrollers;

import frc.robot.subsystems.shooter.shooterrollers.ShooterRollerConstants.ShooterRollerStates;

public abstract class ShooterRollerIO {
  protected double tempCelsius = 0.0;
  protected double statorCurrentAmps = 0.0;
  protected double supplyCurrentAmps = 0.0;

  protected double appliedVolts = 0.0;
  protected double velocity = 0.0;
  protected double desiredVelocity = 0.0;
  protected boolean atSetSpeed = false;
  protected boolean isNoteDetected = false;
  protected ShooterRollerConstants.ShooterRollerStates state = ShooterRollerStates.STOP;

  public void updateInputs() {}

  public void stop() {}

  public void setVelocity(ShooterRollerStates state) {}

  public double getVelocity() {
    return velocity;
  }
}