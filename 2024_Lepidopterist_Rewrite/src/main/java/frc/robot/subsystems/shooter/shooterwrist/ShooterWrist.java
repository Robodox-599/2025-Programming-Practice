package frc.robot.subsystems.shooter.shooterwrist;


public class ShooterWrist {
  private final ShooterWristIO io;

  public ShooterWrist(ShooterWristIO io) {
    this.io = io;
  }

  public void periodic(){
    io.updateInputs();
  }

  public void setAngle(ShooterWristConstants.ShooterWristStates state) {
    io.setAngle(state);
  }

  public boolean isAtSetpoint() {
    return io.atSetpoint;
  }
  
  public void stop() {
    io.stop();
  }
}