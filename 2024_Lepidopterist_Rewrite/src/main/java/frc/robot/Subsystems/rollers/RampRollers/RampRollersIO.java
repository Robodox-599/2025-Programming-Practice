package frc.robot.Subsystems.rollers.RampRollers;

public abstract class RampRollersIO {
    protected double position = 0.0;
    protected double velocity = 0.0;
    protected double statorCurrent = 0.0;
    protected double supplyCurrent = 0.0;
    protected double appliedVolts = 0.0;
    protected double tempCelsius = 0.0;

    protected boolean isCoralDetected = false;
    protected double heldCurrentPosition = 0.0;

    public void updateInputs() {}

    public void setVelocity(double velocity) {}

    public void setPosition(double position) {}

    public void stop() {}
}
