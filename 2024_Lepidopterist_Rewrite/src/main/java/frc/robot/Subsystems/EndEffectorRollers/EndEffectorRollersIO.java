package frc.robot.Subsystems.EndEffectorRollers;

public abstract class EndEffectorRollersIO {
    protected double position = 0.0;
    protected double velocity = 0.0;
    protected double wantedCoralPosition = 0.0;

    protected boolean isCoralDetected = false;

    public void updateInputs(){}
    public void stop() {}
    public void setVelocity(double velocity) {}
    public void setPosition(double position) {}
    
    public double getPosition() {
        return 0.0;
    }
}
