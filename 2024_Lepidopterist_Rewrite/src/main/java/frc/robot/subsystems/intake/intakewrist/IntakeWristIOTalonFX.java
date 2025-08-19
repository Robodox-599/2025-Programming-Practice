// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.intakewrist;

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
import frc.robot.util.PhoenixUtil;
import frc.robot.util.SubsystemUtil;
import static frc.robot.subsystems.intake.intakewrist.IntakeWristConstants.*;

public class IntakeWristIOTalonFX extends IntakeWristIO {

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
        new MotionMagicVoltage(SubsystemUtil.intakeWristStateToSetpoint(IntakeWristStates.STOW))
            .withSlot(0)
            .withEnableFOC(true);

    wristConfig.MotionMagic.MotionMagicCruiseVelocity = maxWristVelocity;
    wristConfig.MotionMagic.MotionMagicAcceleration = maxWristAccel;

    wristConfig.Slot0.kP = realkP;
    wristConfig.Slot0.kI = realkI;
    wristConfig.Slot0.kD = realkD;
    wristConfig.Slot0.kV = realkV;
    wristConfig.Slot0.kS = realkS;
    wristConfig.Slot0.kG = realkG;

    wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    wristConfig.CurrentLimits.SupplyCurrentLimit = 40;
    wristConfig.CurrentLimits.StatorCurrentLimit = 60;
    wristConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    wristConfig.Feedback.RotorToSensorRatio = gearRatio;

    PhoenixUtil.tryUntilOk(10, () -> wristMotor.getConfigurator().apply(wristConfig, 1));
    wristMotor.optimizeBusUtilization();
    position = wristMotor.getPosition();
    velocity = wristMotor.getVelocity();
    appliedVolts = wristMotor.getMotorVoltage();
    current = wristMotor.getStatorCurrent();
    temperature = wristMotor.getDeviceTemp();
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, temperature, velocity, position, current, appliedVolts);
  }

  @Override
  public void updateInputs() {
    BaseStatusSignal.refreshAll(
        position, temperature, velocity, position, current, appliedVolts);
    super.appliedVolts = appliedVolts.getValueAsDouble();
    super.currentAmps = current.getValueAsDouble();
    super.velocity = velocity.getValueAsDouble();
    super.currentPositionDegrees = position.getValueAsDouble();
    super.tempCelsius = temperature.getValueAsDouble();
    super.atSetpoint =
        Math.abs(super.currentPositionDegrees - super.targetPosition) < wristPositionTolerance;

    DogLog.log("IntakeWrist/AppliedVoltage", super.appliedVolts);
    DogLog.log("IntakeWrist/CurrentAmps", super.currentAmps);
    DogLog.log("IntakeWrist/Velocity", super.velocity);
    DogLog.log("IntakeWrist/Temperature", super.tempCelsius);
    DogLog.log("IntakeWrist/CurrentPosition", super.currentPositionDegrees);
    DogLog.log("IntakeWrist/WristAtSetpoint", super.atSetpoint);
    DogLog.log("IntakeWrist/TargetPosition", targetPosition);
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
  public void setAngle(IntakeWristStates state) { 
    double position =
        MathUtil.clamp(SubsystemUtil.intakeWristStateToSetpoint(state), wristMinAngle, wristMaxAngle);
    super.targetPosition = position;
    mmRequest.withPosition(position);
    wristMotor.setControl(mmRequest);
  }
}