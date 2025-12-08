package frc.robot.subsystems.rollers.ramprollers;

public abstract class RampRollersIO {
    
    public void updateInputs() {};
    public void setVelocity(double velocity) {};
    public void setPosition(double position) {};
    public void stop() {};
    // public double holdPosition() {
    //     return holdPosition;
    // }
    
    protected double position = 0;
    protected double velocity = 0;
    protected double holdPosition = 0;
    protected boolean isCoralDetected = false;
}
