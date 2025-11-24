package frc.robot.Subsystems.RampRollers;

public abstract class RampRollersIO {

    //changes over time as it runs
    //use protected only in IO and when defining values
    protected double position = 0.0;
    protected double velocity = 0.0;
    protected double statorCurrent = 0.0;
    protected double supplyCurrent = 0.0;
    protected double wantedCoralPosition = 0.0;

    protected boolean isCoralDetected = false;

    public void updateInputs(){}

    public void setVelocity(double velocity) {}

    public void stop() {}

    public void setPosition(double position) {}

    public double getPosition() {
        return 0.0;
    }
}
