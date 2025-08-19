package frc.robot.subsystems.intake.intakewrist;


public class IntakeWrist {
  private final IntakeWristIO io;

  public IntakeWrist(IntakeWristIO io) {
    this.io = io;
  }

  public void periodic(){
    io.updateInputs();
  }

  public void setAngle(IntakeWristConstants.IntakeWristStates state) {
    io.setAngle(state);
  }

  public boolean isAtSetpoint() {
    return io.atSetpoint;
  }
  
  public void stop() {
    io.stop();
  }
}