// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wristRollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;

//only file that knows TalonFX (the motor) or CANcoder is
public class WristRollersIOTalonFX extends WristRollersIO{
    private final TalonFX wristRollersMotor;

    private final TalonFXConfiguration wristRollersConfig;

    //dont forget to initialize, and add it to refresh after (@BaseStatusSignal)
    private final StatusSignal<AngularVelocity> wristRollersVelocityRad;
    private final StatusSignal<Temperature> wristRollersTemperature;
    private final StatusSignal<Angle> wristRollersPositionRad;



    public WristRollersIOTalonFX(){
        wristRollersMotor = new TalonFX(WristRollersConstants.wristRollersMotorID, WristRollersConstants.wristRollersCANBus);
        wristRollersConfig = new TalonFXConfiguration();

        //specific settings (slots) within general settings
        wristRollersConfig.Slot0.kP = WristRollersConstants.kP;
        wristRollersConfig.Slot0.kI = WristRollersConstants.kI;
        wristRollersConfig.Slot0.kD = WristRollersConstants.kD;
        wristRollersConfig.Slot0.kS = WristRollersConstants.kS;
        wristRollersConfig.Slot0.kG = WristRollersConstants.kG;

        wristRollersMotor.getConfigurator().apply(wristRollersConfig);
        wristRollersMotor.setNeutralMode(NeutralModeValue.Brake);

        wristRollersVelocityRad = wristRollersMotor.getVelocity();
        wristRollersTemperature = wristRollersMotor.getDeviceTemp();
        wristRollersPositionRad = wristRollersMotor.getPosition();

        BaseStatusSignal.setUpdateFrequencyForAll(50, wristRollersVelocityRad, wristRollersPositionRad, wristRollersTemperature);

        //must be applied last
        wristRollersMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(){
        BaseStatusSignal.setUpdateFrequencyForAll(50, wristRollersVelocityRad, wristRollersPositionRad, wristRollersTemperature);
        super.positionRad = wristRollersPositionRad.getValueAsDouble();
        super.velocityRadPerSec = wristRollersPositionRad.getValueAsDouble();

        DogLog.log("WristRollers/PositionRad", super.positionRad);
        DogLog.log("WristRollers/VelocityRadPerSec", super.velocityRadPerSec);
    }

    @Override
    public void stop(){
        wristRollersMotor.stopMotor();
    }

    @Override
    public void setPosition(double position){
        wristRollersMotor.setControl(new PositionDutyCycle(position));
    }

    @Override
    public void setVelocity(double velocity){
        wristRollersMotor.set(velocity);
    }
}
