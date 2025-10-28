package frc.robot.subsystems.intake.intakerollers;

import frc.robot.subsystems.intake.intakerollers.IntakeRollerConstants.IntakeRollerStates;

public abstract class IntakeRollerIO {
  protected double tempCelsius = 0.0;
  protected double statorCurrentAmps = 0.0;
  protected double supplyCurrentAmps = 0.0;

  protected double appliedVolts = 0.0;
  protected double velocity = 0.0;
  protected double desiredVelocity = 0.0;
  protected boolean atSetSpeed = false;
  protected boolean isNoteDetected = false;
  protected IntakeRollerConstants.IntakeRollerStates state = IntakeRollerStates.STOP;

  protected boolean isIntakeRollersStopped = false;

  public void updateInputs() {}

  public void stop() {}

  public void setVelocity(IntakeRollerStates state) {}

  public double getVelocity() {
    return velocity;
  }
}