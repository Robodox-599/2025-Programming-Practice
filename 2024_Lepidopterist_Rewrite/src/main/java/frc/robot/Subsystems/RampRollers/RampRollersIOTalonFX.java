package frc.robot.subsystems.ramprollerS;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
// import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;

public class RampRollersIOTalonFX extends RampRollersIO {
    private final TalonFX rampRollersMotor;
    TalonFXConfiguration rampRollersConfig;
    private DigitalInput rampBeamBreak;
    // private Debouncer rampDebouncer = new Debouncer(0.2);

    private final StatusSignal<AngularVelocity> rampRollersVelocityRad;
    private final StatusSignal<Temperature> rampRollersTemperature;
    private final StatusSignal<Angle> rampRollersPosition;
    private final StatusSignal<Voltage> rampRollersAppliedVolts;
    private final StatusSignal<Current> rampRollersStatorCurrent;
    private final StatusSignal<Current> rampRollersSupplyCurrent;

    private AsynchronousInterrupt beamBreakInterrupt;

    public RampRollersIOTalonFX() {
        rampRollersMotor = new TalonFX(RampRollersConstants.rampRollersMotorID, RampRollersConstants.rampRollersCANBus);
        rampRollersConfig = new TalonFXConfiguration();
        rampBeamBreak = new DigitalInput(RampRollersConstants.beamBreakPort);
        
        rampRollersConfig.Slot0.kP = RampRollersConstants.kP;
        rampRollersConfig.Slot0.kP = RampRollersConstants.kI;
        rampRollersConfig.Slot0.kP = RampRollersConstants.kD;
        rampRollersConfig.Slot0.kP = RampRollersConstants.kS;
        rampRollersConfig.Slot0.kP = RampRollersConstants.kV;

        rampRollersConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rampRollersConfig.CurrentLimits.SupplyCurrentLimit = RampRollersConstants.supplyCurrentLimit;

        rampRollersMotor.getConfigurator().apply(rampRollersConfig);
        rampRollersMotor.setNeutralMode(NeutralModeValue.Brake);

        rampRollersVelocityRad = rampRollersMotor.getVelocity();
        rampRollersTemperature = rampRollersMotor.getDeviceTemp();
        rampRollersAppliedVolts = rampRollersMotor.getMotorVoltage();
        rampRollersPosition = rampRollersMotor.getPosition();
        rampRollersStatorCurrent = rampRollersMotor.getStatorCurrent();
        rampRollersSupplyCurrent = rampRollersMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(50,
        rampRollersVelocityRad, rampRollersTemperature, rampRollersAppliedVolts, rampRollersPosition,
        rampRollersStatorCurrent, rampRollersSupplyCurrent);

        beamBreakInterrupt = new AsynchronousInterrupt(rampBeamBreak, (rising, falling) -> {
            if (falling) {
                super.holdPosition = rampRollersMotor.getPosition().getValueAsDouble();
            }
        });

        beamBreakInterrupt.enable();

        beamBreakInterrupt.setInterruptEdges(true, true);

        rampRollersMotor.optimizeBusUtilization();
    }
    
    @Override
    public void updateInputs() {
        BaseStatusSignal.refreshAll(rampRollersVelocityRad, rampRollersTemperature, rampRollersAppliedVolts, rampRollersPosition,
        rampRollersStatorCurrent, rampRollersSupplyCurrent);
        super.position = rampRollersPosition.getValueAsDouble();
        super.velocity = rampRollersVelocityRad.getValueAsDouble();
        super.isCoralDetected = !rampBeamBreak.get();
        DogLog.log("RampRollers/Position", super.position);
        DogLog.log("RampRollers/Velocity", super.velocity);
        DogLog.log("RampRollers/isCoralDetected", super.isCoralDetected);
        DogLog.log("RampRollers/HoldPosition", super.holdPosition);
    }

    @Override
    public void stop() {
        rampRollersMotor.stopMotor();
    }

    @Override
    public void setPosition(double position) {
        rampRollersMotor.setControl(new PositionDutyCycle(position));
    }

    @Override
    public void setVelocity(double velocity) {
        rampRollersMotor.set(velocity);
    }

    // @Override
    // public double holdPosition() {
    //     if(isCoralDetected) {
    //         super.holdPosition = super.position;
           
    //         return super.holdPosition;
    //     } else {
    //         return super.holdPosition;
    //     }
    // }
}
