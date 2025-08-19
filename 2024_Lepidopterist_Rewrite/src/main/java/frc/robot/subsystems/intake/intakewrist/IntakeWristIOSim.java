// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.intakewrist;

import static frc.robot.subsystems.intake.intakewrist.IntakeWristConstants.*;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.intake.intakewrist.IntakeWristConstants.IntakeWristStates;

public class IntakeWristIOSim extends IntakeWristIO {

  // wrist sim motor + simPID setup
  private static final DCMotor WRIST_GEARBOX = DCMotor.getKrakenX60Foc(1);
  private final DCMotorSim wristSim;

  private PIDController wristPID = new PIDController(simkP, simkI, simkD);

  public IntakeWristIOSim() {
    wristSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(WRIST_GEARBOX, wristMOI, gearRatio), WRIST_GEARBOX);

    wristPID = new PIDController(simkP, simkI, simkD);
    wristPID.setTolerance(wristPositionTolerance);
  }

  @Override
  public void updateInputs() {
    // constantly updating the actual loggging variables for DogLog
    wristSim.update(0.02);

    super.atSetpoint = wristPID.atSetpoint();
    super.appliedVolts = wristSim.getInputVoltage();
    super.currentAmps = wristSim.getCurrentDrawAmps();
    super.velocity = wristSim.getAngularVelocityRPM() / 60.0;
    super.targetPosition = targetPosition;
    super.currentPositionDegrees = wristSim.getAngularPositionRotations();
    super.tempCelsius = 25.0;

    // basic logging for the sim motor
    DogLog.log("IntakeWrist/CurrentAmps", super.currentAmps);
    DogLog.log("IntakeWrist/AppliedVoltage", super.appliedVolts);
    DogLog.log("IntakeWrist/TargetPosition", super.targetPosition);
    DogLog.log("IntakeWrist/CurrentPosition", super.currentPositionDegrees);
    DogLog.log("IntakeWrist/State", super.state.toString());
    DogLog.log("IntakeWrist/Temperature", super.tempCelsius);

    wristSim.setInputVoltage(
        wristPID.calculate(super.currentPositionDegrees, super.targetPosition));
  }

  // allows us to make the wrist motor use whatever custom voltage we want
  @Override
  public void setVoltage(double voltage) {
    wristSim.setInputVoltage(voltage);
  }

  // stop function to stop the motor when we want
  @Override
  public void stop() {
    wristSim.setAngularVelocity(0);
  }

  // Updates the angle of the wrist depending on the state we are moving to
  @Override
  public void setAngle(IntakeWristStates state) {
    targetPosition =
        MathUtil.clamp(IntakeWristConstants.intakeWristSetpoints[state.getIndex()], wristMinAngle, wristMaxAngle);
    wristSim.setInputVoltage(wristPID.calculate(targetPosition));
  }
}