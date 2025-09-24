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
  private final DCMotorSim topFlywheelSimMotor;
  private final DCMotorSim bottomFlywheelSimMotor;
  private PIDController flywheelsController = new PIDController(simkP, simkI, simkD);
  private static final DCMotor INDEXER_GEARBOX = DCMotor.getKrakenX60Foc(1);

  public ShooterFlywheelsIOSim() {
    topFlywheelSimMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(INDEXER_GEARBOX, indexerMOI, gearRatio),
            INDEXER_GEARBOX);
    bottomFlywheelSimMotor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(INDEXER_GEARBOX, indexerMOI, gearRatio),
          INDEXER_GEARBOX);
    flywheelsController =
        new PIDController(simkP, simkI, simkD);
  }

  @Override
  public void updateInputs() {
    topFlywheelSimMotor.update(0.02);
    bottomFlywheelSimMotor.update(0.02);

    
    super.topAtSetSpeed = flywheelsController.atSetpoint();
    super.topAppliedVolts = topFlywheelSimMotor.getInputVoltage();
    super.topStatorCurrentAmps = topFlywheelSimMotor.getCurrentDrawAmps();
    super.topVelocity = topFlywheelSimMotor.getAngularVelocityRPM() / 60.0;
    super.topTempCelsius = 25.0;

    super.bottomAtSetSpeed = flywheelsController.atSetpoint();
    super.bottomAppliedVolts = bottomFlywheelSimMotor.getInputVoltage();
    super.bottomStatorCurrentAmps = bottomFlywheelSimMotor.getCurrentDrawAmps();
    super.bottomVelocity = bottomFlywheelSimMotor.getAngularVelocityRPM() / 60.0;
    super.bottomTempCelsius = 25.0;

    DogLog.log("Flywheels/Top/Velocity", super.topVelocity);
    DogLog.log("Flywheels/Top/Voltage", super.topAppliedVolts);
    DogLog.log("Flywheels/Top/StatorCurrentAmps", super.topStatorCurrentAmps);
    DogLog.log("Flywheels/Top/AtSetSpeed", super.topAtSetSpeed);
    DogLog.log("Flywheels/Top/Temp", 60);

    DogLog.log("Flywheels/Bottom/Velocity", super.bottomVelocity);
    DogLog.log("Flywheels/Bottom/Voltage", super.bottomAppliedVolts);
    DogLog.log("Flywheels/Bottom/StatorCurrentAmps", super.bottomStatorCurrentAmps);
    DogLog.log("Flywheels/Bottom/AtSetSpeed", super.bottomAtSetSpeed);
    DogLog.log("Flywheels/Bottom/Temp", 60);

    DogLog.log("Flywheels/State", super.state.toString());
  }

  @Override
  public void stop() {
    topFlywheelSimMotor.setAngularVelocity(0);
    bottomFlywheelSimMotor.setAngularVelocity(0);
  }

  @Override
  public void setVelocity(ShooterFlywheelsConstants.ShooterFlywheelsStates state) {
    double velocity = SubsystemUtil.shooterFlywheelStateToVelocity(state);
    topFlywheelSimMotor.setAngularVelocity(velocity);
    bottomFlywheelSimMotor.setAngularVelocity(-velocity);
  }
}