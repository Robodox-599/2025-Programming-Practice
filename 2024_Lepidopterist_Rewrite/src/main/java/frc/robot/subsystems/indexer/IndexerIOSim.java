package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexerConstants.gearRatio;
import static frc.robot.subsystems.indexer.IndexerConstants.indexerMOI;
import static frc.robot.subsystems.indexer.IndexerConstants.simkD;
import static frc.robot.subsystems.indexer.IndexerConstants.simkI;
import static frc.robot.subsystems.indexer.IndexerConstants.simkP;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import frc.robot.subsystems.indexer.IndexerConstants.IndexerStates;
import frc.robot.util.SubsystemUtil;

public class IndexerIOSim extends IndexerIO {
  private final DCMotorSim IndexerSim;
  private PIDController indexerController = new PIDController(simkP, simkI, simkD);
  private static final DCMotor INDEXER_GEARBOX = DCMotor.getKrakenX60Foc(1);

  public IndexerIOSim() {
    IndexerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(INDEXER_GEARBOX, indexerMOI, gearRatio),
            INDEXER_GEARBOX);
    indexerController =
        new PIDController(simkP, simkI, simkD);
  }

  @Override
  public void updateInputs() {
    // constantly updating the actual loggging variables for DogLog
    IndexerSim.update(0.02);

    super.atSetSpeed = indexerController.atSetpoint();
    super.appliedVolts = IndexerSim.getInputVoltage();
    super.statorCurrentAmps = IndexerSim.getCurrentDrawAmps();
    super.velocity = IndexerSim.getAngularVelocityRPM() / 60.0;
    super.tempCelsius = 25.0;

    // basic logging for the sim motor
    DogLog.log("Indexer/VelocitySetpoint", desiredVelocity);
    DogLog.log("Indexer/Velocity", super.velocity);
    DogLog.log("Indexer/Voltage", super.appliedVolts);
    DogLog.log("Indexer/StatorCurrentAmps", super.statorCurrentAmps);
    DogLog.log("Indexer/AtSetSpeed", super.atSetSpeed);
    DogLog.log("Indexer/State", super.state.toString());
    DogLog.log("Indexer/Temp", 60);
  }

  // stop function to stop the motor when we want
  @Override
  public void stop() {
    IndexerSim.setAngularVelocity(0);
  }

  // Updates the velocity of the motor depending on the state we are moving to
  @Override
  public void setVelocity(IndexerStates state) {
    double velocity = SubsystemUtil.indexerStateToVelocity(state);
    IndexerSim.setAngularVelocity(velocity);
  }
}