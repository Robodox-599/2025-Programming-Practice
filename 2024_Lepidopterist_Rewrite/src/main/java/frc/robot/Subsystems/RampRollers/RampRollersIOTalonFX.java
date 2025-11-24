package frc.robot.Subsystems.RampRollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;

public class RampRollersIOTalonFX extends RampRollersIO {
    private final TalonFX rampRollersMotor;
    private final TalonFXConfiguration rampRollersConfig;
    private final DigitalInput rampBeamBreak;
    private final Debouncer rampDebouncer;

    private final StatusSignal<AngularVelocity> rampRollersVelocityRad;
    private final StatusSignal<Temperature> rampRollersTemperature;
    private final StatusSignal<Voltage> rampRollersAppliedVolts;
    private final StatusSignal<Angle> rampRollersPosition;
    // stator current, supply current
    // both are StatusSignal<Current>
    // .getSupplyCurrent or .getStatorCurrent

    private AsynchronousInterrupt beamBreakInterrupt;

    public RampRollersIOTalonFX() {
        rampRollersMotor = new TalonFX(RampRollersConstants.rampRollersMotorID, RampRollersConstants.rampRollersCANBus);
        rampRollersConfig = new TalonFXConfiguration();
        rampBeamBreak = new DigitalInput(RampRollersConstants.beamBreakPort);
        rampDebouncer = new Debouncer(RampRollersConstants.rampRollersDebounceTimeSeconds);

        rampRollersConfig.Slot0.kP = RampRollersConstants.kP;
        rampRollersConfig.Slot0.kI = RampRollersConstants.kI;
        rampRollersConfig.Slot0.kD = RampRollersConstants.kD;
        rampRollersConfig.Slot0.kS = RampRollersConstants.kS;
        rampRollersConfig.Slot0.kV = RampRollersConstants.kV;

        rampRollersConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rampRollersConfig.CurrentLimits.SupplyCurrentLimit = RampRollersConstants.supplyCurrentLimit;

        rampRollersMotor.getConfigurator().apply(rampRollersConfig);

        rampRollersMotor.setNeutralMode(NeutralModeValue.Brake);

        rampRollersVelocityRad = rampRollersMotor.getVelocity();
        rampRollersTemperature = rampRollersMotor.getDeviceTemp();
        rampRollersAppliedVolts = rampRollersMotor.getMotorVoltage();
        rampRollersPosition = rampRollersMotor.getPosition();

        BaseStatusSignal.setUpdateFrequencyForAll(50, rampRollersVelocityRad, rampRollersTemperature, rampRollersAppliedVolts, rampRollersPosition);

        beamBreakInterrupt = new AsynchronousInterrupt(rampBeamBreak, (rising, falling) -> {
            if (rising) { // coral -> no coral
                // d
            } 
            if (falling) { // no coral -> coral
                // sdadawd
            }
        });

        rampRollersMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs() {
        BaseStatusSignal.refreshAll(rampRollersVelocityRad, rampRollersTemperature, rampRollersAppliedVolts, rampRollersPosition);
        super.position = rampRollersPosition.getValueAsDouble();
        super.velocity = rampRollersVelocityRad.getValueAsDouble();
        // do the rest

        super.isCoralDetected = rampDebouncer.calculate(!rampBeamBreak.get());

        DogLog.log("RampRollers/Position", super.position);
        DogLog.log("RampRollers/Velocity", super.velocity);
        DogLog.log("RampRollers/isCoralDetected", super.isCoralDetected);
    }

    @Override 
    public void stop() {
        rampRollersMotor.stopMotor();
    }

    @Override
    public void setVelocity(double velocity) {
        rampRollersMotor.set(velocity);
    }

    @Override
    public void setPosition(double position) {
        rampRollersMotor.setControl(new PositionDutyCycle(position));
    }
}
