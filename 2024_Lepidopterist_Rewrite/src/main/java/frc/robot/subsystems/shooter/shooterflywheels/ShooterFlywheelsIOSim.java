package frc.robot.subsystems.shooter.shooterflywheels;

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

public class ShooterFlywheelsIOSim extends ShooterFlywheelsIO {
  private final DCMotorSim ClockwiseSimMotor;
  private final DCMotorSim CounterClockwiseSimMotor;
  private PIDController flywheelsController = new PIDController(simkP, simkI, simkD);
  private static final DCMotor INDEXER_GEARBOX = DCMotor.getKrakenX60Foc(1);

  public ShooterFlywheelsIOSim() {
    ClockwiseSimMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(INDEXER_GEARBOX, indexerMOI, gearRatio),
            INDEXER_GEARBOX);
    CounterClockwiseSimMotor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(INDEXER_GEARBOX, indexerMOI, gearRatio),
          INDEXER_GEARBOX);
    flywheelsController =
        new PIDController(simkP, simkI, simkD);
  }

  @Override
  public void updateInputs() {
    IndexerSim.update(0.02);

    super.atSetSpeed = flywheelsController.atSetpoint();

    super.appliedVolts = IndexerSim.getInputVoltage();
    super.statorCurrentAmps = IndexerSim.getCurrentDrawAmps();
    super.velocity = IndexerSim.getAngularVelocityRPM() / 60.0;
    super.tempCelsius = 25.0;

    DogLog.log("Flywheels/VelocitySetpoint", desiredVelocity);
    DogLog.log("Flywheels/Velocity", super.velocity);
    DogLog.log("Flywheels/Voltage", super.appliedVolts);
    DogLog.log("Flywheels/StatorCurrentAmps", super.statorCurrentAmps);
    DogLog.log("Flywheels/AtSetSpeed", super.atSetSpeed);
    DogLog.log("Flywheels/State", super.state.toString());
    DogLog.log("Flywheels/Temp", 60);
  }

  @Override
  public void stop() {
    IndexerSim.setAngularVelocity(0);
  }

  @Override
  public void setVelocity(ShooterFlywheelsConstants.ShooterFlywheelsStates state) {
    double velocity = SubsystemUtil.intakeRollerStateToVelocity(state);
    IndexerSim.setAngularVelocity(velocity);
  }
}