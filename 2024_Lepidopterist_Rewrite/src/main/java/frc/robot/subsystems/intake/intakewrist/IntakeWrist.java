package frc.robot.subsystems.intake.intakewrist;


public class IntakeWrist {
  private final IntakeWristIO io;

  public IntakeWrist(IntakeWristIO io) {
    this.io = io;
  }

  // updates the logging variables periodically
  public void periodic(){
    io.updateInputs();
  }

  // sets the motor to the designated angle
  public void setAngle(IntakeWristConstants.IntakeWristStates state) {
    io.setAngle(state);
  }

  // checks if we're at the designated angle
  public boolean isAtSetpoint() {
    return io.atSetpoint;
  }
  
  // stop the motor and logging
  public void stop() {
    io.stop();
  }
}