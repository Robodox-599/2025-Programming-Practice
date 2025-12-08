package frc.robot.subsystems.rollers.endeffectorrollers;

public abstract class EndEffectorRollersIO {
    protected double position = 0.0;
    protected double velocity = 0.0;
    public double wantedCoralPositionEndEffectorRollers = 0.0;
    public double statorCurrent = 0.0;
    public double supplyCurrent = 0.0;

    public boolean isCoralDetectedEndEffectorRollers = false;

    public void updateInputs(){}
    public void stop() {}
    public void setVelocity(double velocity) {}
    public void setPosition(double position) {}
    
    public double getPosition() {
        return 0.0;
    }

    
}
