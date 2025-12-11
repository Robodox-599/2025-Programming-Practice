// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffectorrollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;

public class EndeffectorRollersIOTalonFX extends EndeffectorRollersIO {
  private final TalonFX endeffectorRollersMotor;
    TalonFXConfiguration endeffectorRollersConfig;
    private DigitalInput endeffectorRollersBeamBreak;
    private final Debouncer algaeDebounce;

    private final StatusSignal<AngularVelocity> endeffectorRollersVelocityRad;
    private final StatusSignal<Temperature> endeffectorRollersTemperature;
    private final StatusSignal<Angle> endeffectorRollersPosition;
    private final StatusSignal<Voltage> endeffectorRollersAppliedVolts;
    private final StatusSignal<Current> endeffectorRollersStatorCurrent;
    private final StatusSignal<Current> endeffectorRollersSupplyCurrent;

    public EndeffectorRollersIOTalonFX() {
        endeffectorRollersMotor = new TalonFX(EndeffectorRollersConstants.endeffectorRollersMotorID, EndeffectorRollersConstants.endeffectorRollersCANBus);
        endeffectorRollersConfig = new TalonFXConfiguration();
        endeffectorRollersBeamBreak = new DigitalInput(EndeffectorRollersConstants.beamBreakPort);
        
        endeffectorRollersConfig.Slot0.kP = EndeffectorRollersConstants.kP;
        endeffectorRollersConfig.Slot0.kP = EndeffectorRollersConstants.kI;
        endeffectorRollersConfig.Slot0.kP = EndeffectorRollersConstants.kD;
        endeffectorRollersConfig.Slot0.kP = EndeffectorRollersConstants.kS;
        endeffectorRollersConfig.Slot0.kP = EndeffectorRollersConstants.kV;
        endeffectorRollersConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        algaeDebounce = new Debouncer(0.2, Debouncer.DebounceType.kRising);

        endeffectorRollersConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        endeffectorRollersConfig.CurrentLimits.SupplyCurrentLimit = EndeffectorRollersConstants.supplyCurrentLimit;

        endeffectorRollersMotor.getConfigurator().apply(endeffectorRollersConfig);
        endeffectorRollersMotor.setNeutralMode(NeutralModeValue.Brake);

        endeffectorRollersVelocityRad = endeffectorRollersMotor.getVelocity();
        endeffectorRollersTemperature = endeffectorRollersMotor.getDeviceTemp();
        endeffectorRollersAppliedVolts = endeffectorRollersMotor.getMotorVoltage();
        endeffectorRollersPosition = endeffectorRollersMotor.getPosition();
        endeffectorRollersStatorCurrent = endeffectorRollersMotor.getStatorCurrent();
        endeffectorRollersSupplyCurrent = endeffectorRollersMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(50,endeffectorRollersVelocityRad, 
            endeffectorRollersTemperature, endeffectorRollersAppliedVolts, endeffectorRollersPosition, endeffectorRollersStatorCurrent, 
                endeffectorRollersSupplyCurrent);

        endeffectorRollersMotor.optimizeBusUtilization();
    }
    
    @Override
    public void updateInputs() {
        BaseStatusSignal.refreshAll(endeffectorRollersVelocityRad, endeffectorRollersTemperature, 
            endeffectorRollersAppliedVolts, endeffectorRollersPosition, endeffectorRollersStatorCurrent, endeffectorRollersSupplyCurrent);

        super.position = endeffectorRollersPosition.getValueAsDouble();
        super.velocity = endeffectorRollersVelocityRad.getValueAsDouble();
        super.supplyCurrent = endeffectorRollersSupplyCurrent.getValueAsDouble();
        super.statorCurrent = endeffectorRollersStatorCurrent.getValueAsDouble();
        super.appliedVolts = endeffectorRollersAppliedVolts.getValueAsDouble();
        super.tempCelsius = endeffectorRollersTemperature.getValueAsDouble();

        super.isCoralDetected = !endeffectorRollersBeamBreak.get();
        super.isAlgaeDetected = algaeDebounce.calculate(super.statorCurrent > 20);

        DogLog.log("EndeffectorRollers/Velocity", super.velocity);
        DogLog.log("EndeffectorRollers/Position", super.position);
        DogLog.log("EndeffectorRollers/SupplyCurrent", super.supplyCurrent);
        DogLog.log("EndeffectorRollers/StatorCurrent", super.statorCurrent);

        DogLog.log("EndeffectorRollers/AppliedVolts", super.appliedVolts);
        DogLog.log("EndeffectorRollers/Temperature", super.tempCelsius);
        DogLog.log("EndeffectorRollers/isCoralDetected", super.isCoralDetected);
        DogLog.log("EndeffectorRollers/isAlgaeDetected", super.isAlgaeDetected);
    }

    @Override
    public void stop() {
        endeffectorRollersMotor.stopMotor();
    }

    @Override
    public void setPosition(double position) {
        endeffectorRollersMotor.setControl(new PositionDutyCycle(position));
    }

    @Override
    public void holdAlgae(double dutyCycle){
        endeffectorRollersMotor.setControl(new DutyCycleOut(dutyCycle));
    }

    @Override
    public double heldCurrentPosition(){
        if(super.isCoralDetected ){
            super.heldCurrentPosition = super.position;
            
            return super.heldCurrentPosition;
        }
        else{
            return super.heldCurrentPosition;
        }
    }

    @Override
    public void setVelocity(double velocity) {
        endeffectorRollersMotor.set(velocity);
    }
}
