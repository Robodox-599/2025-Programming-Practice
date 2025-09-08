package frc.robot.subsystems.intake.wrist;

import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import static frc.robot.subsystems.intake.wrist.IntakeWristConstants.*;

public class IntakeWristIOSim extends IntakeWristIO {
  private final DCMotorSim wristSim;
  private PIDController wristPID = new PIDController(simkP, simkI, simkD);
  private static final DCMotor WRIST_GEARBOX = DCMotor.getKrakenX60Foc(1);

  public IntakeWristIOSim() {
    wristSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(WRIST_GEARBOX, wristMOI, gearRatio), WRIST_GEARBOX);

    wristPID = new PIDController(simkP, simkI, simkD);
  }

  @Override
  public void updateInputs() {
    wristSim.update(0.02);
    super.tempCelsius = 25.0;
    super.targetPosition = targetPosition;
    super.appliedVoltage = wristSim.getInputVoltage();
    super.currentAmps = wristSim.getCurrentDrawAmps();
    super.velocity = wristSim.getAngularVelocityRPM() / 60.0;
    super.currentPosition = wristSim.getAngularPositionRotations();

    DogLog.log("Intake/Wrist/Temperature", super.tempCelsius);
    DogLog.log("Intake/Wrist/CurrentAmps", super.currentAmps);
    DogLog.log("Intake/Wrist/TargetPosition", super.targetPosition);
    DogLog.log("Intake/Wrist/AppliedVoltage", super.appliedVoltage);
    DogLog.log("Intake/Wrist/CurrentPosition", super.currentPosition);
  

    wristSim.setInputVoltage(
        wristPID.calculate(super.currentPosition, super.targetPosition));
  }

  @Override
  public void setVoltage(double voltage) {
    wristSim.setInputVoltage(voltage);
  }

  @Override
  public void stop() {
    wristSim.setAngularVelocity(0);
  }

  @Override
  public void goToAngle(double angle){
    double position = angle;
    targetPosition = position;
    wristSim.setInputVoltage(wristPID.calculate(targetPosition));
  }
}
