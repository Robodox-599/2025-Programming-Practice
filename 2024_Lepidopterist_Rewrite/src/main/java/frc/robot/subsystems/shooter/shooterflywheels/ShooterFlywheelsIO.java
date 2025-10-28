package frc.robot.subsystems.shooter.shooterflywheels;

import frc.robot.subsystems.shooter.shooterflywheels.ShooterFlywheelsConstants;

public abstract class ShooterFlywheelsIO {
  // top flywheel motor
  protected double topAppliedVolts = 0.0;
  protected double topVelocity = 0.0;
  protected double topDesiredVelocity = 0.0;
  protected boolean topAtSetSpeed = false;
  protected boolean topIsNoteDetected = false;
  protected double topTempCelsius = 0.0;
  protected double topStatorCurrentAmps = 0.0;
  protected double topSupplyCurrentAmps = 0.0;
  
  // bottom flywheel motor
  protected double bottomAppliedVolts = 0.0;
  protected double bottomVelocity = 0.0;
  protected double bottomDesiredVelocity = 0.0;
  protected boolean bottomAtSetSpeed = false;
  protected boolean bottomIsNoteDetected = false;
  protected double bottomTempCelsius = 0.0;
  protected double bottomStatorCurrentAmps = 0.0;
  protected double bottomSupplyCurrentAmps = 0.0;

  protected boolean isAtPrepScoreSpeed = false;
  
  protected ShooterFlywheelsConstants.ShooterFlywheelsStates state = ShooterFlywheelsConstants.ShooterFlywheelsStates.STOP;
  
  public void updateInputs() {}

  public void stop() {}

  public void setVelocity(ShooterFlywheelsConstants.ShooterFlywheelsStates state) {}

  public double getTopVelocity() {
    return topVelocity;
  }
}