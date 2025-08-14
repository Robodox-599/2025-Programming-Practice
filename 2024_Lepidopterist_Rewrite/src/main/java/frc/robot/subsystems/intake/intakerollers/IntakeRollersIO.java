package frc.robot.subsystems.intake.intakerollers;

public abstract class IntakeRollersIO {
  protected double tempCelsius = 0.0;
  protected double statorCurrentAmps = 0.0;
  protected double supplyCurrentAmps = 0.0;

  protected double appliedVolts = 0.0;
  protected double velocity = 0.0;
  protected double desiredVelocity = 0.0;
  protected boolean atSetpoint = false;
  protected boolean isNoteDetected = false;
  protected boolean isNoteEnsured = false;

  public void updateInputs() {}

  public void stop() {}

  public void setVelocity(IntakeRollersConstants.RollersStates state) {}

  public double getVelocity() {
    return velocity;
  }
}