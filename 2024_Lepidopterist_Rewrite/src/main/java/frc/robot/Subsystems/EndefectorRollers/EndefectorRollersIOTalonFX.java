package frc.robot.Subsystems.EndefectorRollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;

public class EndefectorRollersIOTalonFX extends EndefectorRollersIO {
    private final TalonFX endefectorRollersMotor;
    TalonFXConfiguration endefectorRollersConfig;
    private DigitalInput endefectorBeamBreak;
    private Debouncer endefectorDebouncer = new Debouncer(0.2);

    private final StatusSignal<AngularVelocity> endefectorRollersVelocityRad;
    private final StatusSignal<Temperature> endefectorRollersTemperature;
    private final StatusSignal<Angle> endefectorRollersPosition;
    private final StatusSignal<Voltage> endefectorRollersAppliedVolts;
    private final StatusSignal<Current> endefectorRollersStatorCurrent;
    private final StatusSignal<Current> endefectorRollersSupplyCurrent;

    public EndefectorRollersIOTalonFX() {
        endefectorRollersMotor = new TalonFX(EndefectorRollersConstants.endefectorRollersMotorID, EndefectorRollersConstants.endefectorRollersCANBus);
        endefectorRollersConfig = new TalonFXConfiguration();
        endefectorBeamBreak = new DigitalInput(EndefectorRollersConstants.endefectorBeamBreakPort);
        
        endefectorRollersConfig.Slot0.kP = EndefectorRollersConstants.kP;
        endefectorRollersConfig.Slot0.kP = EndefectorRollersConstants.kI;
        endefectorRollersConfig.Slot0.kP = EndefectorRollersConstants.kD;
        endefectorRollersConfig.Slot0.kP = EndefectorRollersConstants.kS;
        endefectorRollersConfig.Slot0.kP = EndefectorRollersConstants.kV;

        endefectorRollersConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        endefectorRollersConfig.CurrentLimits.SupplyCurrentLimit = EndefectorRollersConstants.supplyCurrentLimit;

        endefectorRollersMotor.getConfigurator().apply(endefectorRollersConfig);
        endefectorRollersMotor.setNeutralMode(NeutralModeValue.Brake);

        endefectorRollersVelocityRad = endefectorRollersMotor.getVelocity();
        endefectorRollersTemperature = endefectorRollersMotor.getDeviceTemp();
        endefectorRollersAppliedVolts = endefectorRollersMotor.getMotorVoltage();
        endefectorRollersPosition = endefectorRollersMotor.getPosition();
        endefectorRollersStatorCurrent = endefectorRollersMotor.getStatorCurrent();
        endefectorRollersSupplyCurrent = endefectorRollersMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(50,
        endefectorRollersVelocityRad, endefectorRollersTemperature, endefectorRollersAppliedVolts, endefectorRollersPosition,
        endefectorRollersStatorCurrent, endefectorRollersSupplyCurrent);
    }
    
    @Override
    public void updateInputs() {
        BaseStatusSignal.refreshAll(endefectorRollersVelocityRad, endefectorRollersTemperature, endefectorRollersAppliedVolts, endefectorRollersPosition,
        endefectorRollersStatorCurrent, endefectorRollersSupplyCurrent);
        super.position = endefectorRollersPosition.getValueAsDouble();
        super.velocity = endefectorRollersVelocityRad.getValueAsDouble();
        super.isCoralInEndefector = !endefectorBeamBreak.get();
        DogLog.log("EndefectorRollers/Position", super.position);
        DogLog.log("EndefectorRollers/Velocity", super.velocity);
        DogLog.log("EndefectorRollers/isCoralDetected", super.isCoralInEndefector);
        DogLog.log("EndefectorRollers/HoldPosition", super.holdPosition);
    }

    @Override
    public void setVelocity(double velocity) {
        endefectorRollersMotor.set(velocity);
    }

    @Override
    public void stop() {
        endefectorRollersMotor.stopMotor();
    }

    @Override
    public void setPosition(double position) {
        endefectorRollersMotor.setPosition(position);
    }

    @Override
    public double holdPosition() {
        if(isCoralInEndefector) {
            super.holdPosition = super.position;
           
            return super.holdPosition;
        } else {
            return super.holdPosition;
        }
    }
}
