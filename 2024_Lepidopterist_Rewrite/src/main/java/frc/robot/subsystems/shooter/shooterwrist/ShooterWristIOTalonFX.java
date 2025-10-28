// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.shooterwrist;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.shooterwrist.ShooterWristConstants.*;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.SubsystemUtil;

public class ShooterWristIOTalonFX extends SubsystemBase {
  private final TalonFX wristMotor;
  TalonFXConfiguration wristConfig;
  private final MotionMagicVoltage mmRequest;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;
  private final StatusSignal<Temperature> temperature;

  public IntakeWristIOTalonFX() {
    wristMotor = new TalonFX(wristMotorID, wristMotorCANBus);
    wristConfig = new TalonFXConfiguration();
    mmRequest =
        new MotionMagicVoltage(SubsystemUtil.shooterWristStateToSetpoint(ShooterWristStates.STOP))
            .withSlot(0)
            .withEnableFOC(true);
   
    wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    wristConfig.CurrentLimits.SupplyCurrentLimit = 40;
    wristConfig.CurrentLimits.StatorCurrentLimit = 60;
    wristConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    wristConfig.Feedback.RotorToSensorRatio = gearRatio;
    
    wristConfig.MotionMagic.MotionMagicCruiseVelocity = maxWristVelocity;
    wristConfig.MotionMagic.MotionMagicAcceleration = maxWristAccel;

    wristConfig.Slot0.kP = realkP;
    wristConfig.Slot0.kI = realkI;
    wristConfig.Slot0.kD = realkD;
    wristConfig.Slot0.kV = realkV;
    wristConfig.Slot0.kS = realkS;
    wristConfig.Slot0.kG = realkG;

    PhoenixUtil.tryUntilOk(10, () -> wristMotor.getConfigurator().apply(wristConfig, 1));
    wristMotor.optimizeBusUtilization();
    position = wristMotor.getPosition();
    velocity = wristMotor.getVelocity();
    appliedVolts = wristMotor.getMotorVoltage();
    current = wristMotor.getStatorCurrent();
    temperature = wristMotor.getDeviceTemp();
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, temperature, velocity, position, current, appliedVolts);
  }

  @Override
  public void updateInputs() {
    BaseStatusSignal.refreshAll(
        temperature, velocity, position, current, appliedVolts);
    super.appliedVolts = appliedVolts.getValueAsDouble();
    super.currentAmps = current.getValueAsDouble();
    super.velocity = velocity.getValueAsDouble();
    super.currentPositionDegrees = position.getValueAsDouble();
    super.tempCelsius = temperature.getValueAsDouble();
    super.atSetpoint =
        Math.abs(super.currentPositionDegrees - super.targetPosition) < wristPositionTolerance;
    super.isAtPrepScoreSetpoint = (position.GetValueAsDouble() == 0.7f /*prep score setpoint */) ? true : false;

    DogLog.log("ShooterWrist/AppliedVoltage", super.appliedVolts);
    DogLog.log("ShooterWrist/CurrentAmps", super.currentAmps);
    DogLog.log("ShooterWrist/Velocity", super.velocity);
    DogLog.log("ShooterWrist/Temperature", super.tempCelsius);
    DogLog.log("ShooterWrist/CurrentPosition", super.currentPositionDegrees);
    DogLog.log("ShooterWrist/WristAtSetpoint", super.atSetpoint);
    DogLog.log("ShooterWrist/TargetPosition", targetPosition);
    DogLog.log("IntakeWrist/isAtPrepScoreSetpoint", super.isAtPrepScoreSetpoint);
  }

  @Override
  public void setVoltage(double voltage) {
    wristMotor.setVoltage(voltage);
  }

  @Override
  public void stop() {
    wristMotor.stopMotor();
  }

  @Override
  public void setBrake(boolean brake) {
    wristMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setAngle(ShooterWristStates state) { 
    double position =
        MathUtil.clamp(SubsystemUtil.shooterWristStateToSetpoint(state), wristMinAngle, wristMaxAngle);
    super.targetPosition = position;
    mmRequest.withPosition(position);
    wristMotor.setControl(mmRequest);
  }
}
