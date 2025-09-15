package frc.robot.subsystems.shooter.flywheels;
import static frc.robot.subsystems.shooter.flywheels.FlywheelsConstants.*;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FlywheelsIOSim extends FlywheelsIO {
  private final DCMotorSim topFlywheelSim;
  private final DCMotorSim bottomFlywheelSim;
  private PIDController topController = new PIDController(topSimkP, topSimkI, topSimkD);
  private PIDController bottomController = new PIDController(bottomSimkP, bottomSimkI, bottomSimkD);
  private static final DCMotor TOP_FLYWHEEL_GEARBOX = DCMotor.getKrakenX60Foc(1);
  private static final DCMotor BOTTOM_FLYWHEEL_GEARBOX = DCMotor.getKrakenX60Foc(1);


  public FlywheelsIOSim() {
    topFlywheelSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(TOP_FLYWHEEL_GEARBOX, flywheelsMOI, gearRatio),
            TOP_FLYWHEEL_GEARBOX);
    topController =
        new PIDController(topSimkP, topSimkI, topSimkD);
    bottomFlywheelSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(BOTTOM_FLYWHEEL_GEARBOX, flywheelsMOI, gearRatio),
            BOTTOM_FLYWHEEL_GEARBOX);
    bottomController =
        new PIDController(bottomSimkP, bottomSimkI, bottomSimkD);
  }

  @Override
  public void updateInputs() {
    topFlywheelSim.update(0.02);
    bottomFlywheelSim.update(0.02);

    super.topAppliedVoltage = topFlywheelSim.getInputVoltage();
    super.topStatorCurrentAmps = topFlywheelSim.getCurrentDrawAmps();
    super.topVelocity = topFlywheelSim.getAngularVelocityRPM() / 60.0;
    super.topTempCelsius = 25.0;

    super.bottomAppliedVoltage = bottomFlywheelSim.getInputVoltage();
    super.bottomStatorCurrentAmps = bottomFlywheelSim.getCurrentDrawAmps();
    super.bottomVelocity = bottomFlywheelSim.getAngularVelocityRPM() / 60.0;
    super.bottomTempCelsius = 25.0;
    
    DogLog.log("Shooter/Flywheels/VelocitySetpoint", wantedVelocity);

    DogLog.log("Shooter/Flywheels/Top/Voltage", super.topAppliedVoltage);
    DogLog.log("Shooter/Flywheels/Top/Velocity", super.topVelocity);
    DogLog.log("Shooter/Flywheels/Top/StatorCurrentAmps", super.topStatorCurrentAmps);
    DogLog.log("Shooter/Flywheels/Top/Temp", 60);

    DogLog.log("Shooter/Flywheels/Bottom/Voltage", super.bottomAppliedVoltage);
    DogLog.log("Shooter/Flywheels/Bottom/Velocity", super.bottomVelocity);
    DogLog.log("Shooter/Flywheels/Bottom/StatorCurrentAmps", super.bottomStatorCurrentAmps);
    DogLog.log("Shooter/Flywheels/Bottom/Temp", 60);
  }

  @Override
  public void stop() {
    topFlywheelSim.setAngularVelocity(0);
    bottomFlywheelSim.setAngularVelocity(0);
  }

  @Override
  public void setVelocity(double velocity) {
    topFlywheelSim.setAngularVelocity(velocity);
    bottomFlywheelSim.setAngularVelocity(velocity);
  }
}
