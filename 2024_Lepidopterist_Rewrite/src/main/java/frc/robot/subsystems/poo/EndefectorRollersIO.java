package frc.robot.subsystems.endefectorrollerS;

public abstract class EndefectorRollersIO {
    protected double position = 0;
    protected double velocity = 0;
    protected double holdPosition = 0;
    protected boolean isCoralInEndefector = false;
    
    public void updateInputs() {};
    public void setVelocity(double velocity) {};
    public void setPosition(double position) {};
    public void stop() {};
    public double holdPosition() {
        return holdPosition;
    }
}
