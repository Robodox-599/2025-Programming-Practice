package frc.robot.subsystems.endEffectorRollers;

public abstract class EndEffectorRollersIO {
    protected double position = 0.0;
    protected double velocity = 0.0;
    public static double holdCoralPosition = 0.0;
    public double statorCurrent = 0.0;
    public double supplyCurrent = 0.0;

    protected boolean isCoralDetected = false;
    protected boolean isAlgaeDetected = false;


    public void updateInputs(){}
    public void stop() {}
    public void setVelocity(double velocity) {}
    public void setPosition(double position) {}
    public void holdAlgae(){}
    
    public double getPosition() {
        return 0.0;
    }

}
