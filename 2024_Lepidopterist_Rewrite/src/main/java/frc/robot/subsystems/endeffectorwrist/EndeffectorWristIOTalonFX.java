// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffectorwrist;

import static frc.robot.subsystems.endeffectorwrist.EndeffectorWristConstants.*;

import dev.doglog.DogLog;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.units.measure.Temperature;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.AngularVelocity;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

public class EndeffectorWristIOTalonFX extends EndeffectorWristIO {
  private final TalonFX endeffectorWristMotor;
  TalonFXConfiguration endEffectorWristConfig;

  private final StatusSignal<Voltage> endeffectorWristAppliedVolts;
  private final StatusSignal<AngularVelocity> endeffectorWristVelocity;
  private final StatusSignal<Current> endeffectorWristStatorCurrent;
  private final StatusSignal<Current> endeffectorSupplyCurrent;
  private final StatusSignal<Temperature> endeffectorWristTemperature;
  private final StatusSignal<Angle> endeffectorWristPosition;

  private final StatusSignal<Angle> endeffectorWristAbsolutePosition;

  private final MotionMagicVoltage m_request;
  private final CANcoder CANcoder;

  public EndeffectorWristIOTalonFX() {
    endeffectorWristMotor = new TalonFX(endefectorWristMotorID, endefectorWristCANBus);
    endEffectorWristConfig = new TalonFXConfiguration();

    CANcoder = new CANcoder(cancoderID, endefectorWristCANBus);
    m_request = new MotionMagicVoltage(0);

    CANcoderConfiguration CANcoderConfig = new CANcoderConfiguration();
    CANcoderConfig.MagnetSensor.MagnetOffset = cancoderMagnetOffset;
    CANcoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    CANcoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = discontinuityPoint;
    CANcoder.getConfigurator().apply(CANcoderConfig);

    endEffectorWristConfig.Slot0.kP = kP;
    endEffectorWristConfig.Slot0.kI = kI;
    endEffectorWristConfig.Slot0.kD = kD;
    endEffectorWristConfig.Slot0.kS = kS;
    endEffectorWristConfig.Slot0.kV = kV;
    endEffectorWristConfig.Slot0.kG = kG;

    endEffectorWristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    endEffectorWristConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    endEffectorWristConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    endEffectorWristConfig.Feedback.FeedbackRemoteSensorID = cancoderID;
    endEffectorWristConfig.Feedback.RotorToSensorRatio = gearRatio;
    endEffectorWristConfig.ClosedLoopGeneral.ContinuousWrap = false;

    endEffectorWristConfig.MotionMagic.MotionMagicCruiseVelocity = MaxVelocity;
    endEffectorWristConfig.MotionMagic.MotionMagicAcceleration = MaxAcceleration;

    endeffectorWristAppliedVolts = endeffectorWristMotor.getMotorVoltage();
    endeffectorWristVelocity = endeffectorWristMotor.getVelocity();
    endeffectorWristPosition = endeffectorWristMotor.getPosition();
    endeffectorWristTemperature = endeffectorWristMotor.getDeviceTemp();
    endeffectorWristStatorCurrent = endeffectorWristMotor.getStatorCurrent();
    endeffectorSupplyCurrent = endeffectorWristMotor.getStatorCurrent();
    endeffectorWristAbsolutePosition = CANcoder.getAbsolutePosition();

    
    BaseStatusSignal.setUpdateFrequencyForAll(
      50.0, endeffectorWristAppliedVolts, endeffectorWristVelocity, endeffectorWristTemperature, 
      endeffectorWristPosition, endeffectorWristStatorCurrent, endeffectorSupplyCurrent, endeffectorWristAbsolutePosition);
      endeffectorWristMotor.optimizeBusUtilization();
      CANcoder.optimizeBusUtilization();
  } 

  public void updateInputs() {
    BaseStatusSignal.refreshAll(
        endeffectorWristAppliedVolts, endeffectorWristVelocity, endeffectorWristTemperature, 
          endeffectorWristPosition, endeffectorWristStatorCurrent, endeffectorSupplyCurrent, endeffectorWristAbsolutePosition);

    super.appliedVolts = endeffectorWristAppliedVolts.getValueAsDouble();
    super.statorCurrent = endeffectorWristStatorCurrent.getValueAsDouble();
    super.supplyCurrent = endeffectorSupplyCurrent.getValueAsDouble();
    super.velocity = endeffectorWristVelocity.getValueAsDouble();
    super.currentPosition = endeffectorWristPosition.getValueAsDouble();
    super.tempCelsius = endeffectorWristTemperature.getValueAsDouble();

    super.isWristInPosition = MathUtil.isNear(targetPosition, currentPosition, isWristInPositionTolerence);
   
    DogLog.log("Endeffector/Wrist/Velocity", super.velocity);
    DogLog.log("Endeffector/Wrist/StatorCurrent", super.statorCurrent);
    DogLog.log("Endeffector/Wrist/SupplyCurrent", super.supplyCurrent);
    DogLog.log("Endeffector/Wrist/Temperature", super.tempCelsius);
    DogLog.log("Endeffector/Wrist/TargetPosition", targetPosition);
    DogLog.log("Endeffector/Wrist/AppliedVoltage", super.appliedVolts);
    DogLog.log("Endeffector/Wrist/CurrentPosition", super.currentPosition);
    DogLog.log("Endeffector/Wrist/isWristInPosition", super.isWristInPosition);
    DogLog.log("Endeffector/Wrist/AbsolutePosition", endeffectorWristAbsolutePosition.getValueAsDouble());
  }

  @Override
  public void setPosition(double angle) {
    super.targetPosition = angle;
    endeffectorWristMotor.setControl(m_request);
    m_request.withPosition(angle);
  }

  @Override
  public void stop(){
    endeffectorWristMotor.stopMotor();
  }
}
