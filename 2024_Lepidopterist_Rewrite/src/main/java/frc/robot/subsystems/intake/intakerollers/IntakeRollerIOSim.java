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
import frc.robot.subsystems.intake.intakerollers.IntakeRollerConstants.IntakeRollerStates;
import frc.robot.util.SubsystemUtil;

public class IntakeRollerIOSim extends IntakeRollerIO {
  private final DCMotorSim IndexerSim;
  private PIDController indexerController = new PIDController(simkP, simkI, simkD);
  private static final DCMotor INDEXER_GEARBOX = DCMotor.getKrakenX60Foc(1);

  public IntakeRollerIOSim() {
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

    super.atSetSpeed = indexerController.atSetpoint();

    super.appliedVolts = IndexerSim.getInputVoltage();
    super.statorCurrentAmps = IndexerSim.getCurrentDrawAmps();
    super.velocity = IndexerSim.getAngularVelocityRPM() / 60.0;
    super.tempCelsius = 25.0;

    DogLog.log("IntakeRollers/VelocitySetpoint", desiredVelocity);
    DogLog.log("IntakeRollers/Velocity", super.velocity);
    DogLog.log("IntakeRollers/Voltage", super.appliedVolts);
    DogLog.log("IntakeRollers/StatorCurrentAmps", super.statorCurrentAmps);
    DogLog.log("IntakeRollers/AtSetSpeed", super.atSetSpeed);
    DogLog.log("IntakeRollers/State", super.state.toString());
    DogLog.log("IntakeRollers/Temp", 60);
  }

  @Override
  public void stop() {
    IndexerSim.setAngularVelocity(0);
  }

  @Override
  public void setVelocity(IntakeRollerStates state) {
    double velocity = SubsystemUtil.intakeRollerStateToVelocity(state);
    IndexerSim.setAngularVelocity(velocity);
  }
}