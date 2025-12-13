package frc.robot.subsystems.endefectorwrist;

public abstract class EndefectorWristIO {
    protected double position = 0;
    protected double velocity = 0;
    protected double wantedPosition = 0;
    protected boolean isWristInPosition = false;
    
    public void updateInputs() {};
    public void setVelocity(double velocity) {};
    public void setPosition(double position) {};
    public void stop() {};
}
