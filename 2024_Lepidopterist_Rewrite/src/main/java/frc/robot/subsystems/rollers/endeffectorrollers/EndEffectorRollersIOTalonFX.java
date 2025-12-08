package frc.robot.subsystems.rollers.endeffectorrollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;

public class EndEffectorRollersIOTalonFX extends EndEffectorRollersIO {
    //shows error if it isn't defined
    private final TalonFX endEffectorRollersMotor;

    //settings of the motor (i.e. sets the limits of current, speed, & PID values)
    private final TalonFXConfiguration endEffectorRollersConfig;
    private final DigitalInput endEffectorBeamBreak;

    //Initialize, and add it to refresh after
    private final StatusSignal<AngularVelocity> endEffectorRollersVelocityRad;
    private final StatusSignal<Temperature> endEffectorRollersTemperature;
    private final StatusSignal<Voltage> endEffectorRollersAppliedVolts;
    private final StatusSignal<Angle> endEffectorRollersPosition;
    private final StatusSignal<Current> endEffectorRollersStatorCurrent;
    private final StatusSignal<Current> endEffectorRollersSupplyCurrent;

    public EndEffectorRollersIOTalonFX(){
        endEffectorRollersMotor = new TalonFX(EndEffectorRollersConstants.EndEffectorRollersMotorID, EndEffectorRollersConstants.EndEffectorRollersCANBus);
        endEffectorRollersConfig = new TalonFXConfiguration();
        endEffectorBeamBreak = new DigitalInput(EndEffectorRollersConstants.beamBreakPort);

        endEffectorRollersConfig.Slot0.kP = EndEffectorRollersConstants.kP;
        endEffectorRollersConfig.Slot0.kI = EndEffectorRollersConstants.kI;
        endEffectorRollersConfig.Slot0.kD = EndEffectorRollersConstants.kD;
        endEffectorRollersConfig.Slot0.kS = EndEffectorRollersConstants.kS;
        endEffectorRollersConfig.Slot0.kV = EndEffectorRollersConstants.kV;

        endEffectorRollersConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        endEffectorRollersConfig.CurrentLimits.SupplyCurrentLimit = EndEffectorRollersConstants.supplyCurrentLimit;

        endEffectorRollersMotor.getConfigurator().apply(endEffectorRollersConfig);
        endEffectorRollersMotor.setNeutralMode(NeutralModeValue.Brake);

        endEffectorRollersVelocityRad = endEffectorRollersMotor.getVelocity();
        endEffectorRollersTemperature = endEffectorRollersMotor.getDeviceTemp();
        endEffectorRollersAppliedVolts = endEffectorRollersMotor.getMotorVoltage();
        endEffectorRollersPosition = endEffectorRollersMotor.getPosition();
        endEffectorRollersStatorCurrent = endEffectorRollersMotor.getStatorCurrent();
        endEffectorRollersSupplyCurrent = endEffectorRollersMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(50, endEffectorRollersVelocityRad, endEffectorRollersTemperature, endEffectorRollersAppliedVolts, endEffectorRollersPosition, endEffectorRollersSupplyCurrent, endEffectorRollersStatorCurrent);

        endEffectorRollersMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(){
        BaseStatusSignal.refreshAll(endEffectorRollersVelocityRad, endEffectorRollersTemperature, endEffectorRollersAppliedVolts, endEffectorRollersPosition, endEffectorRollersSupplyCurrent, endEffectorRollersStatorCurrent);

        super.position = endEffectorRollersPosition.getValueAsDouble();
        super.velocity = endEffectorRollersVelocityRad.getValueAsDouble();
        super.isCoralDetectedEndEffectorRollers = !endEffectorBeamBreak.get();
        super.statorCurrent = endEffectorRollersStatorCurrent.getValueAsDouble();
        super.supplyCurrent = endEffectorRollersSupplyCurrent.getValueAsDouble();

        DogLog.log("endEffectorRollers/Position", super.position);
        DogLog.log("endEffectorRollers/Velocity", super.velocity);
        DogLog.log("endEffectorRollers/isCoralDetected", super.isCoralDetectedEndEffectorRollers);
        DogLog.log("endEffectorRollers/statorCurrent", super.statorCurrent);
        DogLog.log("endEffectorRollers/supplyCurrent", super.supplyCurrent);

    }

    @Override
    public void stop(){
        endEffectorRollersMotor.stopMotor();
    }

    @Override
    public void setVelocity(double velocity){
        endEffectorRollersMotor.set(velocity);
    }
    
    @Override
    public void setPosition(double position){
        endEffectorRollersMotor.setControl(new PositionDutyCycle(position));
    }

    @Override
    public double getPosition(){
        return endEffectorRollersMotor.getPosition().getValueAsDouble();
    }

    public void endEffectorRollerStop(){
        endEffectorRollersMotor.stopMotor();
    }
}
