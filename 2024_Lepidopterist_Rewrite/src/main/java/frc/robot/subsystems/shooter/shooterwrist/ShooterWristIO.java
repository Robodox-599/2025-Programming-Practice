package frc.robot.subsystems.shooter.shooterwrist;

import frc.robot.subsystems.intake.intakewrist.IntakeWristConstants.WristStates;

public abstract class ShooterWristIO {
  protected double tempCelsius = 0.0;
  protected double currentAmps = 0.0;
  protected double appliedVolts = 0.0;
  protected double velocity = 0.0;
  protected double targetPosition = 0.0;
  protected double currentPositionDegrees = 0.0;
  protected boolean atSetpoint = false;

  public void updateInputs() {}

  public void setVoltage(double voltage) {}

  public void goToPose(double position) {}

  public void stop() {}

  public void setBrake(boolean brake) {}

  public void setAngle(WristStates state) {}

  public double getCurrentVolts() {
    return appliedVolts;
  }
}