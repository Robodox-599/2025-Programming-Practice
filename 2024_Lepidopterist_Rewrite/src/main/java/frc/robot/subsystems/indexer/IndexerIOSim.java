package frc.robot.subsystems.indexer;
import static frc.robot.subsystems.indexer.IndexerConstants.*;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IndexerIOSim extends IndexerIO {
  private final DCMotorSim indexerSim;
  private PIDController indexerController = new PIDController(simkP, simkI, simkD);
  private static final DCMotor INDEXER_GEARBOX = DCMotor.getKrakenX60Foc(1);

  public IndexerIOSim() {
    indexerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(INDEXER_GEARBOX, indexerMOI, gearRatio),
            INDEXER_GEARBOX);
    indexerController =
        new PIDController(simkP, simkI, simkD);
  }

  @Override
  public void updateInputs() {
    indexerSim.update(0.02);

    super.appliedVoltage = indexerSim.getInputVoltage();
    super.statorCurrentAmps = indexerSim.getCurrentDrawAmps();
    super.velocity = indexerSim.getAngularVelocityRPM() / 60.0;
    super.tempCelsius = 25.0;

    DogLog.log("Indexer/Voltage", super.appliedVoltage);
    DogLog.log("Indexer/Velocity", super.velocity);
    DogLog.log("Indexer/VelocitySetpoint", wantedVelocity);
    DogLog.log("Indexer/StatorCurrentAmps", super.statorCurrentAmps);
    DogLog.log("Indexer/Temp", 60);
  }

  @Override
  public void stop() {
    indexerSim.setAngularVelocity(0);
  }

  @Override
  public void setVelocity(double velocity) {
    indexerSim.setAngularVelocity(velocity);
  }
}
