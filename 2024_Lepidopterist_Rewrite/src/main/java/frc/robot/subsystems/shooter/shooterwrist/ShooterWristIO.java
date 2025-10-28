
package frc.robot.subsystems.shooter.shooterwrist;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.shooterwrist.ShooterWristConstants.ShooterWristStates;

public class ShooterWristIO extends SubsystemBase {
  protected double tempCelsius = 0.0;
  protected double currentAmps = 0.0;
  protected double appliedVolts = 0.0;
  protected double velocity = 0.0;
  protected double targetPosition = 0.0;
  protected double currentPositionDegrees = 0.0;
  protected boolean atSetpoint = false;
  protected ShooterWristConstants.ShooterWristStates state = ShooterWristStates.STOP;

  protected boolean isAtPrepScoreSetpoint = false;

  public void updateInputs() {}

  public void setVoltage(double voltage) {}

  public void goToPose(double position) {}

  public void stop() {}

  public void setBrake(boolean brake) {}

  public void setAngle(ShooterWristConstants.ShooterWristStates state) {}

  public double getCurrentVolts() {
    return appliedVolts;
  }
}
