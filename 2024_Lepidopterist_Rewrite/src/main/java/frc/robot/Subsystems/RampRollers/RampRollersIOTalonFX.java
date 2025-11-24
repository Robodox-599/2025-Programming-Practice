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
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;

public class RampRollersIOTalonFX extends RampRollersIO {
    //shows error if it isn't defined
    private final TalonFX rampRollersMotor;
    //settings of the motor (i.e. sets the limits of current, speed, & PID values)
    private final TalonFXConfiguration rampRollersConfig;
    private final DigitalInput rampBeamBreak;
    private final Debouncer rampDebouncer;

    //Initialize, and add it to refresh after
    private final StatusSignal<AngularVelocity> rampRollersVelocityRad;
    private final StatusSignal<Temperature> rampRollersTemperature;
    private final StatusSignal<Voltage> rampRollersAppliedVolts;
    private final StatusSignal<Angle> rampRollersPosition;
    //(Homework) also include stator current/ supply current
    //both are StatusSignal<Current>
    //add variables to IO
    // .getSupplyCurrent or .getStatorCurrent
    private final StatusSignal<Current> rampRollersStatorCurrent;
    private final StatusSignal<Current> rampRollersSupplyCurrent;

    private AsynchronousInterrupt beamBreakInterrupt;

    public RampRollersIOTalonFX() {
        rampRollersMotor = new TalonFX(RampRollersConstants.rampRollersMotorID, RampRollersConstants.rampRollersCANBus);
        rampRollersConfig = new TalonFXConfiguration();
        rampBeamBreak = new DigitalInput(RampRollersConstants.beamBreakPort);
        rampDebouncer = new Debouncer(RampRollersConstants.rampRollersDebounceTimeSeconds);

        //general settings is Config, and within those specific settings there are different options which are slots i.e. crosshair profiles
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
        // stator current
        rampRollersStatorCurrent = rampRollersMotor.getStatorCurrent();
        // supply current
        rampRollersSupplyCurrent = rampRollersMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(50, rampRollersVelocityRad, rampRollersTemperature, rampRollersAppliedVolts, rampRollersPosition, rampRollersSupplyCurrent, rampRollersStatorCurrent);

        
        beamBreakInterrupt = new AsynchronousInterrupt(rampBeamBreak, (rising,falling) -> {
            if (falling){
                super.wantedCoralPosition = getPosition();
            }
            //rising going from false to true
            //falling going true to false
        });

        //this must be applied LAST
        rampRollersMotor.optimizeBusUtilization();
    }

    @Override 
    public void updateInputs() {
        BaseStatusSignal.refreshAll(rampRollersVelocityRad, rampRollersTemperature, rampRollersAppliedVolts, rampRollersPosition, rampRollersStatorCurrent, rampRollersSupplyCurrent);
        //super gets the thing from the parent in this case RampRollersIO
        super.position = rampRollersPosition.getValueAsDouble();
        super.velocity = rampRollersVelocityRad.getValueAsDouble();

        //calculate => waits the debounceTime, if true for the duration of the debounceTime, then it's set to true
        super.isCoralDetected = rampDebouncer.calculate(!rampBeamBreak.get());

        super.statorCurrent = rampRollersStatorCurrent.getValueAsDouble();
        super.supplyCurrent = rampRollersSupplyCurrent.getValueAsDouble();

        DogLog.log("RampRollers/Position", super.position);
        DogLog.log("RampRollers/Velocity", super.velocity);
        DogLog.log("RampRollers/isCoralDetected", super.isCoralDetected);
        DogLog.log("RampRollers/statorCurrent", super.statorCurrent);
        DogLog.log("RampRollers/supplyCurrent", super.supplyCurrent);
    }
    
    @Override
    public void stop(){
        rampRollersMotor.stopMotor();
    }

    @Override
    // parameters needs to match that of it defined in the parent
    public void setVelocity(double velocity){
        rampRollersMotor.set(velocity);
    }

    //"set" sets the speed, "setControl" is applicable to a variety of things, however setControl needs to have a request (sometimes importing the PositionDutyCycle needs to be done manually)
    @Override
    public void setPosition(double position){
        rampRollersMotor.setControl(new PositionDutyCycle(position));
    }

    @Override
    public double getPosition(){
        return rampRollersMotor.getPosition().getValueAsDouble();
    }
}
