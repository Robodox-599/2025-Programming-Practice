package frc.robot.subsystems.shooter.shooterrollers;

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
import frc.robot.subsystems.shooter.shooterrollers.ShooterRollerConstants.ShooterRollerStates;
import frc.robot.util.SubsystemUtil;

public class ShooterRollerIOSim extends ShooterRollerIO {
  private final DCMotorSim IndexerSim;
  private PIDController indexerController = new PIDController(simkP, simkI, simkD);
  private static final DCMotor INDEXER_GEARBOX = DCMotor.getKrakenX60Foc(1);

  public ShooterRollerIOSim() {
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

    DogLog.log("ShooterRollers/VelocitySetpoint", desiredVelocity);
    DogLog.log("ShooterRollers/Velocity", super.velocity);
    DogLog.log("ShooterRollers/Voltage", super.appliedVolts);
    DogLog.log("ShooterRollers/StatorCurrentAmps", super.statorCurrentAmps);
    DogLog.log("ShooterRollers/AtSetSpeed", super.atSetSpeed);
    DogLog.log("ShooterRollers/State", super.state.toString());
    DogLog.log("ShooterRollers/Temp", 60);
  }

  @Override
  public void stop() {
    IndexerSim.setAngularVelocity(0);
  }

  @Override
  public void setVelocity(ShooterRollerStates state) {
    double velocity = SubsystemUtil.shooterRollerStateToVelocity(state);
    IndexerSim.setAngularVelocity(velocity);
  }
}