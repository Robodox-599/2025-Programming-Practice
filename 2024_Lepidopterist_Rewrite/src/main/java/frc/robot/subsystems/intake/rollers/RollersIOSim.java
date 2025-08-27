package frc.robot.subsystems.intake.rollers;
import static frc.robot.subsystems.intake.rollers.RollersConstants.*;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class RollersIOSim extends RollersIO {
  private final DCMotorSim rollersSim;
  private PIDController rollersController = new PIDController(simkP, simkI, simkD);
  private static final DCMotor ROLLERS_GEARBOX = DCMotor.getKrakenX60Foc(1);

  public RollersIOSim() {
    rollersSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(ROLLERS_GEARBOX, rollersMOI, gearRatio),
            ROLLERS_GEARBOX);
    rollersController =
        new PIDController(simkP, simkI, simkD);
  }

  @Override
  public void updateInputs() {
    rollersSim.update(0.02);

    super.appliedVoltage = rollersSim.getInputVoltage();
    super.statorCurrentAmps = rollersSim.getCurrentDrawAmps();
    super.velocity = rollersSim.getAngularVelocityRPM() / 60.0;
    super.tempCelsius = 25.0;

    DogLog.log("Intake/Rollers/Voltage", super.appliedVoltage);
    DogLog.log("Intake/Rollers/Velocity", super.velocity);
    DogLog.log("Intake/Rollers/StatorCurrentAmps", super.statorCurrentAmps);
    DogLog.log("Intake/Rollers/Temp", 60);
  }

  @Override
  public void stop() {
    rollersSim.setAngularVelocity(0);
  }

  @Override
  public void setVelocity(double velocity) {
    rollersSim.setAngularVelocity(velocity);
  }
}
