// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.shooterwrist;

import static frc.robot.subsystems.intake.intakewrist.IntakeWristConstants.*;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.shooter.shooterwrist.ShooterWristConstants.ShooterWristStates;

public class ShooterWristIOSim extends ShooterWristIO {
  private static final DCMotor WRIST_GEARBOX = DCMotor.getKrakenX60Foc(1);
  private final DCMotorSim wristSim;

  private PIDController wristPID = new PIDController(simkP, simkI, simkD);

  public ShooterWristIOSim() {
    wristSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(WRIST_GEARBOX, wristMOI, gearRatio), WRIST_GEARBOX);

    wristPID = new PIDController(simkP, simkI, simkD);
    wristPID.setTolerance(wristPositionTolerance);
  }

  @Override
  public void updateInputs() {
    wristSim.update(0.02);

    super.atSetpoint = wristPID.atSetpoint();
    super.appliedVolts = wristSim.getInputVoltage();
    super.currentAmps = wristSim.getCurrentDrawAmps();
    super.velocity = wristSim.getAngularVelocityRPM() / 60.0;
    super.targetPosition = targetPosition;
    super.currentPositionDegrees = wristSim.getAngularPositionRotations();
    super.tempCelsius = 25.0;

    DogLog.log("ShooterWrist/CurrentAmps", super.currentAmps);
    DogLog.log("ShooterWrist/AppliedVoltage", super.appliedVolts);
    DogLog.log("ShooterWrist/TargetPosition", super.targetPosition);
    DogLog.log("ShooterWrist/CurrentPosition", super.currentPositionDegrees);
    DogLog.log("ShooterWrist/State", super.state.toString());
    DogLog.log("ShooterWrist/Temperature", super.tempCelsius);

    wristSim.setInputVoltage(
        wristPID.calculate(super.currentPositionDegrees, super.targetPosition));
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
  public void setAngle(ShooterWristStates state) {
    targetPosition =
        MathUtil.clamp(ShooterWristConstants.shooterWristSetpoints[state.getIndex()], wristMinAngle, wristMaxAngle);
    wristSim.setInputVoltage(wristPID.calculate(targetPosition));
  }
}