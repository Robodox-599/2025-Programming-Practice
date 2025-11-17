package frc.robot.Subsystems.RampRollers;

public abstract class RampRollersIO {
    protected double position = 0.0;
    protected double velocity = 0.0;

    protected boolean isCoralDetected = false;
    protected double heldCurrentPosition = 0.0;

    public void updateInputs() {}

    public void setVelocity(double velocity) {}

    public void setPosition(double position) {}

    public double heldCurrentPosition() {
        return heldCurrentPosition;
    }

    public void stop() {}
}
