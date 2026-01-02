// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endEffectorWrist;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

//only file that knows TalonFX (the motor) or CANcoder is
public class EndEffectorWristIOTalonFX extends EndEffectorWristIO{
    private final TalonFX endEffectorWristMotor;
    private final CANcoder endEffectorWristCANCoder;
    private final CANcoderConfiguration CANcoderConfig;
    private final TalonFXConfiguration endEffectorWristConfig;
    MotionMagicVoltage m_request;

    //dont forget to initialize, and add it to refresh after (@BaseStatusSignal)
    private final StatusSignal<AngularVelocity> endEffectorWristVelocityRad;
    private final StatusSignal<Temperature> endEffectorWristTemperature;
    private final StatusSignal<Angle> endEffectorWristPositionRad;
    private final StatusSignal<Current> endEffectorWristStatorCurrent;
    private final StatusSignal<Current> endEffectorWristSupplyCurrent;
    private final StatusSignal<Voltage> endEffectorWristAppliedVolts;

    public EndEffectorWristIOTalonFX(){
        endEffectorWristMotor = new TalonFX(EndEffectorWristConstants.endEffectorWristMotorID, EndEffectorWristConstants.endEffectorWristCANBus);
        endEffectorWristConfig = new TalonFXConfiguration();
        endEffectorWristCANCoder = new CANcoder(EndEffectorWristConstants.endEffectorWristCANCoderID);
        CANcoderConfig = new CANcoderConfiguration();
        m_request = new MotionMagicVoltage(0);

        CANcoderConfig.MagnetSensor.MagnetOffset = EndEffectorWristConstants.endEffectorWristMagnetOffset;
        CANcoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        CANcoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = EndEffectorWristConstants.absoluteDiscontinuityPoint;
        endEffectorWristCANCoder.getConfigurator().apply(CANcoderConfig);

        //specific settings (slots) within general settings
        //configurations control the math the motor performs after it reads the sensor
        endEffectorWristConfig.Slot0.kP = EndEffectorWristConstants.kP;
        endEffectorWristConfig.Slot0.kI = EndEffectorWristConstants.kI;
        endEffectorWristConfig.Slot0.kD = EndEffectorWristConstants.kD;
        endEffectorWristConfig.Slot0.kS = EndEffectorWristConstants.kS;
        endEffectorWristConfig.Slot0.kG = EndEffectorWristConstants.kG;
        endEffectorWristConfig.Slot0.kV = EndEffectorWristConstants.kV;
        //When wrist is fully extended parallel to the ground gravitational force applies most torque. When wrist end is above pivot, pivot supports the weight of the endEffector.
        //Cosine function determines this. Cos(0) = 1, cos(90) = 0
        endEffectorWristConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        endEffectorWristConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        endEffectorWristConfig.CurrentLimits.SupplyCurrentLimit = EndEffectorWristConstants.supplyCurrentLimit;

        //confugres the robot's sensors so it knows its physical location
        //when robot turns on, it's zero'd to where it last was, CANcoder tells the absolute 0.
        //feedback sensor source tells the device what position/angle it's at
        endEffectorWristConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        endEffectorWristConfig.Feedback.FeedbackRemoteSensorID = EndEffectorWristConstants.endEffectorWristCANCoderID;
        //Motor to Wrist Gear Ratio is different, defines a full rotation for the motor in terms of the wrist gear
        endEffectorWristConfig.Feedback.RotorToSensorRatio = EndEffectorWristConstants.endEffectorWristGearRatio;
        // Disables the arm from wrapping around becasue the arm has physical limits (360degrees is not 0)
        endEffectorWristConfig.ClosedLoopGeneral.ContinuousWrap = false;

        //motion magic settings
        endEffectorWristConfig.MotionMagic.MotionMagicCruiseVelocity = EndEffectorWristConstants.endEffectorWristMaxVelocity;
        endEffectorWristConfig.MotionMagic.MotionMagicAcceleration = EndEffectorWristConstants.endEffectorWristMaxAcceleration;

        //?????
        endEffectorWristMotor.getConfigurator().apply(endEffectorWristConfig);
        endEffectorWristCANCoder.getConfigurator().apply(CANcoderConfig);
        endEffectorWristMotor.setNeutralMode(NeutralModeValue.Brake);

        endEffectorWristVelocityRad = endEffectorWristMotor.getVelocity();
        endEffectorWristTemperature = endEffectorWristMotor.getDeviceTemp();
        endEffectorWristPositionRad = endEffectorWristMotor.getPosition();
        endEffectorWristSupplyCurrent = endEffectorWristMotor.getSupplyCurrent();
        endEffectorWristStatorCurrent = endEffectorWristMotor.getStatorCurrent();
        endEffectorWristAppliedVolts = endEffectorWristMotor.getMotorVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(50, endEffectorWristVelocityRad, endEffectorWristPositionRad, endEffectorWristTemperature, endEffectorWristSupplyCurrent, endEffectorWristStatorCurrent, endEffectorWristAppliedVolts);

        //must be applied last
        endEffectorWristMotor.optimizeBusUtilization();
        endEffectorWristCANCoder.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(){
        BaseStatusSignal.refreshAll(endEffectorWristVelocityRad, endEffectorWristPositionRad, endEffectorWristTemperature, endEffectorWristSupplyCurrent, endEffectorWristStatorCurrent, endEffectorWristAppliedVolts);
        super.positionRad = endEffectorWristPositionRad.getValueAsDouble();
        super.velocityRadPerSec = endEffectorWristVelocityRad.getValueAsDouble();

        DogLog.log("endEffectorWrist/PositionRad", super.positionRad);
        DogLog.log("endEffectorWrist/VelocityRadPerSec", super.velocityRadPerSec);
    }

    @Override
    public void stop(){
        endEffectorWristMotor.stopMotor();
    }

    @Override
    public void setPosition(double position){
        endEffectorWristMotor.setControl(m_request.withPosition(position));
    }
}
