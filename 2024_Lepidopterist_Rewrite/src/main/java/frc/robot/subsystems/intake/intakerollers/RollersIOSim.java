package frc.robot.subsystems.intake.intakerollers;

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

import frc.robot.subsystems.intake.intakerollers.RollersConstants.RollersStates;
import frc.robot.util.SubsystemUtil;

public class RollersIOSim extends RollersIO {
  private final DCMotorSim IndexerSim;
  private PIDController indexerController = new PIDController(simkP, simkI, simkD);
  private static final DCMotor INDEXER_GEARBOX = DCMotor.getKrakenX60Foc(1);

  public RollersIOSim() {
    IndexerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(INDEXER_GEARBOX, indexerMOI, gearRatio),
            INDEXER_GEARBOX);
    indexerController =
        new PIDController(simkP, simkI, simkD);
  }

  @Override
  public void updateInputs() {
    IndexerSim.update(0.02);

    super.appliedVolts = IndexerSim.getInputVoltage();
    super.statorCurrentAmps = IndexerSim.getCurrentDrawAmps();
    super.velocity = IndexerSim.getAngularVelocityRPM() / 60.0;
    super.tempCelsius = 25.0;

    DogLog.log("Rollers/VelocitySetpoint", desiredVelocity);
    DogLog.log("Rollers/Velocity", super.velocity);
    DogLog.log("Rollers/Voltage", super.appliedVolts);
    DogLog.log("Rollers/StatorCurrentAmps", super.statorCurrentAmps);
    DogLog.log("Rollers/Temp", 60);
  }

  @Override
  public void stop() {
    IndexerSim.setAngularVelocity(0);
  }

  @Override
  public void setVelocity(RollersStates state) {
    double velocity = SubsystemUtil.rollersStateToVelocity(state);
    IndexerSim.setAngularVelocity(velocity);
  }
}